// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * VisionSubsystem for Team 6875 — 2026 season.
 *
 * Architecture based on FRC 3940's 2025 VisionSubsystem:
 *   - updateCamera() pattern (called once per periodic)
 *   - cosine trick for selecting the best-facing target (avoids dot product edge cases)
 *   - Pose estimation fed into CTRE drivetrain via addVisionMeasurement()
 *   - Field2d display for Elastic/Shuffleboard
 *
 * Hardware:
 *   Single PhotonVision camera: "AprilTag_Camera" on OrangePi 5
 *   Camera position: 8" forward, 8.75" left of center, 19" high, yaw = 0°
 *
 * 2026 Tags used:
 *   Tower (Red):  15, 16
 *   Tower (Blue): 31, 32
 *   Goal  (Red):  9, 10, 8, 5, 11, 2
 *   Goal  (Blue): 25, 26, 18, 27, 21, 24
 *
 * NOTE: Tag positions are loaded from the WPILib field layout.
 *       Update AprilTagFields.kDefaultField once FIRST releases the 2026 layout.
 */
public class VisionSubsystem extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // Constants
    // ─────────────────────────────────────────────────────────────────────────
    public static class VisionConstants {

        // Camera name — must match exactly what's configured in PhotonVision UI
        public static final String kCameraName = "AprilTag_Camera";

        // Camera physical position relative to robot center (robot-centric frame)
        // +X = forward, +Y = left of center, +Z = up
        // Yaw = 0° means camera faces straight forward
        public static final Transform3d kRobotToCamera = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.75),    // 8" forward of center GRACE FOREMAN**** 13.75- DITANCE FORM FRONT EDGES
                Units.inchesToMeters(9.4375),   // 8.75" left of center  13.75- DITANCE FORM LEFT EDGES
                Units.inchesToMeters(20.25)    // 19" above floor
            ),
            new Rotation3d(0.0, 10.0, 0.0)    // Camera faces straight forward
        );

        // How much to trust vision measurements when updating pose estimator
        // Lower values = more trust. Multi-tag is more reliable than single-tag.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.9, 0.9, 1.5);
        public static final Matrix<N3, N1> kMultiTagStdDevs  = VecBuilder.fill(0.3, 0.3, 0.9);

        // Reject single-tag poses with ambiguity above this threshold
        // (0.2 is a good starting point; lower = stricter)
        public static final double kMaxAmbiguity = 0.2;

        // ── 2026 tag IDs ──────────────────────────────────────────────────────
        // Tower tags — used for climbing alignment
        public static final int[] kRedTowerTagIDs  = {15, 16};
        public static final int[] kBlueTowerTagIDs = {31, 32};

        // Goal tags — used for shooting alignment
        // Red:  straight {9,10} | left-angle {8,5} | right-angle {11,2}
        // Blue: straight {25,26} | left-angle {18,27} | right-angle {21,24}
        public static final int[] kRedGoalTagIDs  = {9, 10, 8, 5, 11, 2};
        public static final int[] kBlueGoalTagIDs = {25, 26, 18, 27, 21, 24};

        // Angular tolerance for cosine-based target selection.
        // A tag is "preferred" if its normal is within this angle of the robot's
        // rear-facing direction. Matches 3940's 25° tolerance.
        public static final double kAngTolerance = Units.degreesToRadians(25.0);

        // Maximum distance (meters) from a tower tag before AlignToTowerCommand
        // will refuse to activate. Prevents accidental mid-match alignment.
        // TUNABLE — change this if 2m feels too tight or too loose.
        public static final double kMaxTowerAlignDistance = 2.0; // meters

        // Per-tag fine-tuning offsets for alignment.
        // Array index = tag ID; values = {xLeft, yLeft, xRight, yRight} in meters.
        // All zeros by default — tune these during field calibration.
        // Index 0–32 covers all 2026 field tags.
        public static final double[][] kTagSpecificOffset = new double[33][4];
    }

    // ── Inner class: holds the most recently selected target tag ─────────────
    public class VisionTargetTag {
        public int    tagID = 0;
        public Pose2d pose  = new Pose2d();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Fields
    // ─────────────────────────────────────────────────────────────────────────
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera         camera;
    private final PhotonPoseEstimator  poseEstimator;

    // Reference to drivetrain for pose reading and vision measurement injection
    private final CommandSwerveDrivetrain drivetrain;

    // Field display widget shown on Elastic / Shuffleboard
    private final Field2d fieldDisplay = new Field2d();

    // Pre-built target lists — populated in constructor, never rebuilt
    private final List<Pose2d> blueTowerTargets = new ArrayList<>();
    private final List<Pose2d> redTowerTargets  = new ArrayList<>();
    private final List<Pose2d> blueGoalTargets  = new ArrayList<>();
    private final List<Pose2d> redGoalTargets   = new ArrayList<>();

    // Parallel ID arrays — index i in targets[i] maps to tagIDs[i]
    private int[] blueTowerTagIDs;
    private int[] redTowerTagIDs;
    private int[] blueGoalTagIDs;
    private int[] redGoalTagIDs;

    // Reusable target holder — avoids per-frame allocation
    private final VisionTargetTag currentTarget = new VisionTargetTag();

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Load field layout — update AprilTagFields constant when 2026 layout ships
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Set up camera
        camera = new PhotonCamera(VisionConstants.kCameraName);

        // Multi-tag PnP is most accurate; single-tag fallback uses current pose
        poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kRobotToCamera
        );
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        SmartDashboard.putData("Field", fieldDisplay);

        // ── Build tower target lists ──────────────────────────────────────────
        blueTowerTagIDs = buildTargetList(VisionConstants.kBlueTowerTagIDs, blueTowerTargets);
        redTowerTagIDs  = buildTargetList(VisionConstants.kRedTowerTagIDs,  redTowerTargets);

        // ── Build goal target lists ───────────────────────────────────────────
        blueGoalTagIDs = buildTargetList(VisionConstants.kBlueGoalTagIDs, blueGoalTargets);
        redGoalTagIDs  = buildTargetList(VisionConstants.kRedGoalTagIDs,  redGoalTargets);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Private helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Builds a Pose2d list and returns a parallel int[] of tag IDs.
     * Tags not present in the field layout are silently skipped.
     */
    private int[] buildTargetList(int[] requestedIDs, List<Pose2d> targetList) {
        List<Integer> foundIDs = new ArrayList<>();
        for (int id : requestedIDs) {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(id);
            if (tagPose.isPresent()) {
                targetList.add(tagPose.get().toPose2d());
                foundIDs.add(id);
            }
        }
        return foundIDs.stream().mapToInt(Integer::intValue).toArray();
    }

    /**
     * Reads the latest camera frame and feeds the estimated pose into the
     * CTRE drivetrain's pose estimator via addVisionMeasurement().
     *
     * Mirrors 3940's updateCamera() structure — called once per periodic().
     */
    private void updateCamera() {
        final List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        // Use only the most recent frame to avoid latency build-up
        var latestResult = results.get(results.size() - 1);

        // Reference pose helps single-tag fallback pick the right solution
        poseEstimator.setReferencePose(drivetrain.getState().Pose);

        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(latestResult);

        if (estimatedPose.isPresent()) {
            int numTargets = latestResult.getTargets().size();

            // Reject high-ambiguity single-tag poses — they're often wrong
            if (numTargets == 1
                    && latestResult.getBestTarget().getPoseAmbiguity() > VisionConstants.kMaxAmbiguity) {
                fieldDisplay.getObject("EstimatedPose").setPose(new Pose2d(-100, -100, new Rotation2d()));
                return;
            }

            Pose2d  pose2d    = estimatedPose.get().estimatedPose.toPose2d();
            double  timestamp = estimatedPose.get().timestampSeconds;
            Matrix<N3, N1> stdDevs = (numTargets > 1)
                ? VisionConstants.kMultiTagStdDevs
                : VisionConstants.kSingleTagStdDevs;

            // Inject measurement into the CTRE pose estimator
            drivetrain.addVisionMeasurement(pose2d, timestamp, stdDevs);

            fieldDisplay.getObject("EstimatedPose").setPose(pose2d);
        } else {
            // Move display off-screen so stale marker doesn't linger
            fieldDisplay.getObject("EstimatedPose").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }
    }

    /**
     * 3940's cosine trick — selects the target whose facing direction is
     * most aligned with the robot's rear.
     *
     * Math: cos(tag_angle - robot_rear_angle) > cos(tolerance)
     * is equivalent to: |tag_angle - robot_rear_angle| < tolerance
     * but works correctly across the 0/2π wrap-around.
     *
     * Falls back to nearest-by-distance if no tag is within tolerance.
     *
     * @param targets  List of Pose2d for this alliance's tag group
     * @param tagIDs   Parallel array of tag IDs corresponding to targets
     * @param useFrontFacing
     */
    
     private Pose2d selectClosestTarget(List<Pose2d> targets, int[] tagIDs, boolean useFrontFacing) {
        if (targets.isEmpty()) {
            currentTarget.tagID = 0;
            currentTarget.pose  = new Pose2d();
            return currentTarget.pose;
        }

        // "Rear" = direction the robot's back is pointing
        double robotAngle = drivetrain.getState().Pose.getRotation().getRadians();
        double referenceAngle = useFrontFacing ? robotAngle : robotAngle - Math.PI;

        // First pass: find a tag whose normal is within kAngTolerance of robot rear
        for (int i = 0; i < targets.size(); i++) {
            double tagAngle = targets.get(i).getRotation().getRadians();
            if (Math.cos(tagAngle - referenceAngle) > Math.cos(VisionConstants.kAngTolerance)) {
                currentTarget.tagID = tagIDs[i];
                currentTarget.pose  = targets.get(i);
                return currentTarget.pose;
            }
        }

        // Fallback: pick geometrically nearest tag
        Pose2d nearest = drivetrain.getState().Pose.nearest(targets);
        for (int i = 0; i < targets.size(); i++) {
            if (nearest.getX() == targets.get(i).getX()
                    && nearest.getY() == targets.get(i).getY()) {
                currentTarget.tagID = tagIDs[i];
            }
        }
        currentTarget.pose = nearest;
        return currentTarget.pose;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Public accessors — called by commands
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns the Pose2d of the best tower tag for the given alliance.
     * Also updates currentTarget.tagID for use by getTargetTagID().
     */
    public Pose2d getClosestTowerTarget(boolean isBlue) {
        return isBlue
            ? selectClosestTarget(blueTowerTargets, blueTowerTagIDs, false)
            : selectClosestTarget(redTowerTargets,  redTowerTagIDs, false);
    }

    /**
     * Returns the Pose2d of the best goal tag for the given alliance.
     * Also updates currentTarget.tagID for use by getTargetTagID().
     */
    public Pose2d getClosestGoalTarget(boolean isBlue) {
        return isBlue
            ? selectClosestTarget(blueGoalTargets, blueGoalTagIDs,true)
            : selectClosestTarget(redGoalTargets,  redGoalTagIDs,true);
    }

    /**
     * Returns the tag ID from the most recent getClosest*() call.
     * Use this to look up tag-specific offsets in VisionConstants.kTagSpecificOffset.
     */
    public int getTargetTagID() {
        return currentTarget.tagID;
    }

    /**
     * Returns the straight-line distance (meters) from the robot to the
     * nearest tower tag. Used by AlignToTowerCommand to reject presses
     * when the robot is too far away.
     */
    public double getDistanceToTower(boolean isBlue) {
        Pose2d towerPose = getClosestTowerTarget(isBlue);
        Pose2d robotPose = drivetrain.getState().Pose;
        double dx = towerPose.getX() - robotPose.getX();
        double dy = towerPose.getY() - robotPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Returns the straight-line distance (meters) to the nearest goal tag.
     * Used by AlignToGoalCommand for physics-based speed calculation.
     */
    public double getDistanceToGoal(boolean isBlue) {
        Pose2d goalPose  = getClosestGoalTarget(isBlue);
        Pose2d robotPose = drivetrain.getState().Pose;
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Returns the robot's current field-relative pose from the drivetrain. */
    public Pose2d getRobotPose() {
        return drivetrain.getState().Pose;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystem overrides
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Update pose estimation from the camera every loop
        updateCamera();

        // Keep the field display in sync with actual robot position
        fieldDisplay.setRobotPose(drivetrain.getState().Pose);

        // Publish alliance for easy dashboard viewing
        SmartDashboard.putBoolean("Vision/IsBlue", isBlue());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Utility
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns true if the robot is on the blue alliance.
     * Falls back to false (red) if alliance is not yet reported by the DS.
     */
    public boolean isBlue() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;
    }
}

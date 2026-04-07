// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.util.PathPlannerLogging;

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
 * Uses PhotonVision on an Orange Pi 5 to:
 *   - Continuously update robot pose estimate (fed into the CTRE drivetrain)
 *   - Find the nearest hub AprilTag for shooting alignment
 *   - Show robot position and active path on Elastic / SmartDashboard
 *
 * Camera setup:
 *   The AprilTag camera is FRONT-FACING, mounted on the LEFT SIDE of the robot.
 *   Camera position relative to robot center: TUNE — fill in after measuring.
 *
 * 2026 Hub AprilTag IDs:
 *   Red  — straight: {9, 10}  | side: {8, 5}  | side: {11, 2}
 *   Blue — straight: {25, 26} | side: {18, 27} | side: {21, 24}
 *
 * Based on FRC 3940's 2025 VisionSubsystem structure.
 */
public class VisionForAligningSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────────────────
    public static class VisionConstants {

        // Camera name — must match EXACTLY what's set in the PhotonVision web UI
        public static final String kCameraName = "NewAprilTag_Camera"; // TODO: verify name in PhotonVision UI

        // Camera position relative to robot center (robot-centric frame).
        // +X = forward, +Y = left of center, +Z = up.
        // Pitch is positive tilting up, Yaw is positive rotating left.
        // TUNE: measure these after mounting the camera on your robot.
        public static final Transform3d kRobotToCamera = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.5),   // TODO: forward distance from robot center to camera
                Units.inchesToMeters(9.75),   // TODO: left/right distance (positive = left)
                Units.inchesToMeters(20.95)    // TODO: height above floor
            ),
            new Rotation3d(
                0.0,                            // Roll  — usually 0
                Units.degreesToRadians(20),   // TODO: Pitch (camera tilt angle in degrees)
                0.0                             // Yaw   — 0 for front-facing camera
            )
        );

        // How much to trust vision pose estimates (lower = more trust).
        // Multi-tag is more accurate than single-tag.
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.9, 0.9, 999999999);
        public static final Matrix<N3, N1> kMultiTagStdDevs  = VecBuilder.fill(0.3, 0.3, 999999999);

        // Reject single-tag poses with ambiguity above this threshold.
        // 0.2 is a good starting point (lower = stricter).
        public static final double kMaxAmbiguity = 0.2;

        // Reject any vision measurement that would jump the pose estimate more than
        // this distance in one update.  Prevents a bad AprilTag reading from
        // teleporting the pose and causing PathPlanner to tweak out.
        // Increase if legitimate vision updates are being rejected.
        public static final double kMaxPoseJumpMeters = 1.0; // TUNE

        // 2026 hub AprilTag IDs for each alliance
        // Red:  straight {9,10} | left-angle {8,5} | right-angle {11,2}
        // Blue: straight {25,26} | left-angle {18,27} | right-angle {21,24}
        public static final int[] kRedGoalTagIDs  = {9, 10, 8, 5, 11, 2};
        public static final int[] kBlueGoalTagIDs = {25, 26, 18, 27, 21, 24};

        // Angle tolerance for "closest target" selection.
        // Tags within this many degrees of the reference angle are preferred.
        public static final double kAngTolerance = Units.degreesToRadians(25.0);
    }

    // Placeholder — remove after filling in TUNE values above
    //private static final double TUNE = 0.0;

    // ── Inner class: tracks which tag was last selected ───────────────────────
    public class VisionTargetTag {
        public int    tagID = 0;
        public Pose2d pose  = new Pose2d();
    }

    // ── Fields ────────────────────────────────────────────────────────────────
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera         camera;
    private final PhotonPoseEstimator  poseEstimator;
    private final CommandSwerveDrivetrain drivetrain;

    // Field2d widget displayed on Elastic / SmartDashboard
    private final Field2d fieldDisplay = new Field2d();

    // Pre-built lists of hub tag poses for each alliance (built once in constructor)
    private final List<Pose2d> redGoalTargets  = new ArrayList<>();
    private final List<Pose2d> blueGoalTargets = new ArrayList<>();
    private int[] redGoalTagIDs;
    private int[] blueGoalTagIDs;

    // Most recently selected target (updated by getClosestGoalTarget)
    private final VisionTargetTag currentTarget = new VisionTargetTag();

    // ── Constructor ──────────────────────────────────────────────────────────

    public VisionForAligningSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Load the 2026 field layout
        // NOTE: Update this constant when FIRST releases the official 2026 layout file.
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Set up the camera and pose estimator
        camera = new PhotonCamera(VisionConstants.kCameraName);
        poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kRobotToCamera
        );
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        // Put the field widget on Elastic so we can see robot position in real time
        SmartDashboard.putData("Field", fieldDisplay);

        // Show PathPlanner's planned path on the field widget (helpful for debugging autos)
        PathPlannerLogging.setLogTargetPoseCallback(target ->
            fieldDisplay.getObject("PP Target").setPose(target));
        PathPlannerLogging.setLogActivePathCallback(poses ->
            fieldDisplay.getObject("PP Path").setPoses(poses));

        // Build the goal target lists once at startup
        redGoalTagIDs  = buildTargetList(VisionConstants.kRedGoalTagIDs,  redGoalTargets);
        blueGoalTagIDs = buildTargetList(VisionConstants.kBlueGoalTagIDs, blueGoalTargets);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Private helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Populates a target list from the field layout and returns a parallel int[] of tag IDs.
     * Tags not found in the field layout are silently skipped.
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
     * Reads the latest camera frame and feeds the estimated robot pose
     * into the CTRE drivetrain's pose estimator.
     *
     * This runs during both teleop AND autonomous.
     * Vision measurements during auto help PathPlanner stay on course.
     */
    private void updateCamera() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        // Only use the most recent frame to avoid building up latency
        PhotonPipelineResult latestResult = results.get(results.size() - 1);

        // Give the estimator a reference pose so the single-tag fallback picks the right solution
        poseEstimator.setReferencePose(drivetrain.getState().Pose);

        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(latestResult);

        if (estimatedPose.isPresent()) {
            int numTargets = latestResult.getTargets().size();

            // Reject single-tag poses that are ambiguous (two valid solutions exist)
            if (numTargets == 1
                    && latestResult.getBestTarget().getPoseAmbiguity() > VisionConstants.kMaxAmbiguity) {
                fieldDisplay.getObject("EstimatedPose").setPose(new Pose2d(-100, -100, new Rotation2d()));
                return;
            }

            Pose2d  estimatedPose2d = estimatedPose.get().estimatedPose.toPose2d();
            double  timestamp       = estimatedPose.get().timestampSeconds;
            Matrix<N3, N1> stdDevs  = (numTargets > 1)
                ? VisionConstants.kMultiTagStdDevs
                : VisionConstants.kSingleTagStdDevs;

            // Reject vision measurements that would jump the pose estimate too far.
            // This is the main guard against bad AprilTag readings conflicting with
            // PathPlanner during autonomous — if vision suddenly sees the wrong tag
            // and reports a pose 3 meters away, we throw it out instead of chasing it.
            double poseJump = estimatedPose2d.getTranslation()
                .getDistance(drivetrain.getState().Pose.getTranslation());
            if (poseJump > VisionConstants.kMaxPoseJumpMeters) {
                return; // Reject — measurement is too far from current estimate
            }

            // Inject the vision measurement into the CTRE pose estimator (Kalman filter)
            drivetrain.addVisionMeasurement(estimatedPose2d, timestamp, stdDevs);

            fieldDisplay.getObject("EstimatedPose").setPose(estimatedPose2d);
        } else {
            // Move the display marker off-screen so a stale pose doesn't linger
            fieldDisplay.getObject("EstimatedPose").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }
    }

    /**
     * Selects the best hub tag to target based on robot orientation.
     *
     * Uses 3940's cosine trick: prefers the tag whose outward normal is most
     * aligned with the robot's FRONT direction.  This selects the tag the
     * shooter (back) is pointing at when the robot is aimed correctly.
     *
     * Falls back to the geometrically nearest tag if none pass the angle test.
     */
    private Pose2d selectClosestGoalTarget(List<Pose2d> targets, int[] tagIDs) {
        if (targets.isEmpty()) {
            currentTarget.tagID = 0;
            currentTarget.pose  = new Pose2d();
            return currentTarget.pose;
        }

        // Robot's front direction (the camera is front-facing, shooter is on back)
        double robotFrontAngle = drivetrain.getState().Pose.getRotation().getRadians();

        // First pass: find a tag whose normal faces the same direction as the robot front
        // (This tag is the one the shooter is pointing at when the back faces the hub)
        for (int i = 0; i < targets.size(); i++) {
            double tagAngle = targets.get(i).getRotation().getRadians();
            if (Math.cos(tagAngle - robotFrontAngle) > Math.cos(VisionConstants.kAngTolerance)) {
                currentTarget.tagID = tagIDs[i];
                currentTarget.pose  = targets.get(i);
                return currentTarget.pose;
            }
        }

        // Fallback: pick the geometrically nearest tag to the robot
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
    // Public accessors (called by commands)
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns the field-relative Pose2d of the best hub tag to target.
     * Also sets the tag ID accessible via getTargetTagID().
     *
     * @param isBlue true for blue alliance, false for red
     */
    public Pose2d getClosestGoalTarget(boolean isBlue) {
        return isBlue
            ? selectClosestGoalTarget(blueGoalTargets, blueGoalTagIDs)
            : selectClosestGoalTarget(redGoalTargets,  redGoalTagIDs);
    }

    /** Returns the tag ID from the most recent getClosestGoalTarget() call. */
    public int getTargetTagID() {
        return currentTarget.tagID;
    }

    /**
     * Returns the straight-line distance (meters) from the robot to the
     * nearest hub tag.  Used by SMARTShoot to look up the correct flywheel speed.
     *
     * @param isBlue true for blue alliance, false for red
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

    /**
     * Returns true if the robot is on the blue alliance.
     * Falls back to false (red) if the Driver Station hasn't reported yet.
     */
    public boolean isBlue() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Periodic
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Update robot pose from camera every loop (runs in both teleop and auto)
        updateCamera();

        // Keep the Elastic field widget in sync with actual robot position
        fieldDisplay.setRobotPose(drivetrain.getState().Pose);

        // Publish alliance so you can verify it on Elastic
        SmartDashboard.putBoolean("Vision/IsBlue", isBlue());
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionConstants;

/**
 * AlignToTowerCommand — drives the robot to the tower and holds side-specific
 * alignment while the operator holds the button.
 *
 * Bound to: Operator LT (whileTrue)
 *
 * ── What it does ────────────────────────────────────────────────────────────
 *  1. Safety check: if the robot is farther than kMaxTowerAlignDistance (2 m,
 *     tunable) from the nearest tower tag, the command refuses to run and prints
 *     a warning. This is the "idiot proofing" for accidental mid-match presses.
 *
 *  2. Side detection: determines whether the robot is to the LEFT or RIGHT of
 *     the tower tag using the tag's local Y axis (the cosine/dot-product trick).
 *     - Robot on LEFT  → target robot heading = tag angle + 90° (face left)
 *     - Robot on RIGHT → target robot heading = tag angle − 90° (face right)
 *
 *  3. Drive + rotate: simultaneous ProfiledPIDController for both translation
 *     (X and Y) and rotation (yaw). The robot moves to kTowerBumperDistance
 *     alongside the tag while rotating to the correct side-facing heading.
 *     PID runs continuously — the robot HOLDS position while LT is held.
 *
 *  4. Stopped state: once both translation and rotation are within tolerance,
 *     "TOWER ALIGNED ✅" is displayed on Elastic. The command continues holding
 *     position until LT is released (it never self-terminates).
 *
 * ── Geometry ────────────────────────────────────────────────────────────────
 *  Tag normal  = direction tag faces (outward from tower)
 *  Tag left    = tag normal rotated +90° (CCW) = (-sin θ, cos θ)
 *  Tag right   = tag normal rotated -90° (CW)  = (+sin θ, -cos θ)
 *
 *  If robot is on LEFT of tag:  target = tag_pos + left_tangent * kTowerBumperDistance
 *                                heading = tagAngle + π/2
 *  If robot is on RIGHT of tag: target = tag_pos + right_tangent * kTowerBumperDistance
 *                                heading = tagAngle - π/2
 *
 * ── Tuning guide ────────────────────────────────────────────────────────────
 *  kTowerBumperDistance — increase if robot isn't close enough, decrease if it rams
 *  kMaxTowerAlignDistance — increase if 2 m feels too restrictive
 *  kTransP / kRotP — increase if alignment is slow, decrease if it oscillates
 */
public class AlignToTowerCommand extends Command {

    // ── Tunable constants ─────────────────────────────────────────────────────

    /**
     * Distance from tower tag to robot center when bumper is touching the tower.
     * Approximately: robot_half_width + bumper_thickness ≈ 0.40 m (≈ 16")
     * TUNABLE — adjust this on the real robot.
     */
    private static final double kTowerBumperDistance = 0.40; // meters — TUNABLE

    // Translation PID (X and Y axes use the same gains)
    private static final double kTransP = 2.5;
    private static final double kTransI = 0.0;
    private static final double kTransD = 0.1;
    private static final double kMaxTransVel   = 2.0; // m/s
    private static final double kMaxTransAccel = 2.0; // m/s²

    // Rotation PID
    private static final double kRotP = 2.0;
    private static final double kRotI = 0.0;
    private static final double kRotD = 0.05;
    private static final double kMaxRotVel   = 3.0 * Math.PI; // rad/s
    private static final double kMaxRotAccel = 2.0 * Math.PI; // rad/s²

    // "At target" tolerances
    private static final double kTransTolerance = 0.03;               // meters
    private static final double kRotTolerance   = Math.toRadians(2.0); // radians
    // ─────────────────────────────────────────────────────────────────────────

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem         visionSubsystem;

    private final ProfiledPIDController xController = new ProfiledPIDController(
        kTransP, kTransI, kTransD,
        new TrapezoidProfile.Constraints(kMaxTransVel, kMaxTransAccel)
    );
    private final ProfiledPIDController yController = new ProfiledPIDController(
        kTransP, kTransI, kTransD,
        new TrapezoidProfile.Constraints(kMaxTransVel, kMaxTransAccel)
    );
    private final ProfiledPIDController rotController = new ProfiledPIDController(
        kRotP, kRotI, kRotD,
        new TrapezoidProfile.Constraints(kMaxRotVel, kMaxRotAccel)
    );

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // State set in initialize(), used across execute() calls
    private boolean tooFar       = false; // True if rejected due to distance
    private boolean robotOnLeft  = false; // Side detection result
    private double  targetX      = 0;
    private double  targetY      = 0;
    private double  targetHeading = 0;

    public AlignToTowerCommand(CommandSwerveDrivetrain drivetrain,
                                VisionSubsystem visionSubsystem) {
        this.drivetrain      = drivetrain;
        this.visionSubsystem = visionSubsystem;
        addRequirements(drivetrain);

        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setTolerance(kRotTolerance);
        xController.setTolerance(kTransTolerance);
        yController.setTolerance(kTransTolerance);
    }

    @Override
    public void initialize() {
        boolean isBlue = visionSubsystem.isBlue();

        // ── Safety check: reject if too far from the tower ───────────────────
        double distanceToTower = visionSubsystem.getDistanceToTower(isBlue);
        if (distanceToTower > VisionConstants.kMaxTowerAlignDistance) {
            tooFar = true;
            SmartDashboard.putString("AlignTower/Status",
                "TOO FAR (" + String.format("%.1f", distanceToTower) + " m). Move closer first.");
            return;
        }
        tooFar = false;

        // ── Get nearest tower tag ─────────────────────────────────────────────
        Pose2d tagPose   = visionSubsystem.getClosestTowerTarget(isBlue);
        Pose2d robotPose = drivetrain.getState().Pose;

        double tagAngle = tagPose.getRotation().getRadians();

        // ── Determine which side of the tower the robot is on ────────────────
        // Vector from tag to robot
        double dx = robotPose.getX() - tagPose.getX();
        double dy = robotPose.getY() - tagPose.getY();

        // Tag's left tangent direction (tag normal rotated +90° CCW)
        // If dot product with this vector > 0, robot is on the LEFT side of the tag
        double tagLeftX = -Math.sin(tagAngle);
        double tagLeftY =  Math.cos(tagAngle);
        double sideDot  = dx * tagLeftX + dy * tagLeftY;
        robotOnLeft = sideDot > 0;

        // ── Calculate target position and heading ─────────────────────────────
        if (robotOnLeft) {
            // Robot approaches from left → face left (tag angle + 90°)
            targetHeading = tagAngle + Math.PI / 2.0;
            // Target position: offset from tag in the left-tangent direction
            targetX = tagPose.getX() + tagLeftX * kTowerBumperDistance;
            targetY = tagPose.getY() + tagLeftY * kTowerBumperDistance;
        } else {
            // Robot approaches from right → face right (tag angle - 90°)
            targetHeading = tagAngle - Math.PI / 2.0;
            // Target position: offset from tag in the right-tangent direction
            double tagRightX = Math.sin(tagAngle);
            double tagRightY = -Math.cos(tagAngle);
            targetX = tagPose.getX() + tagRightX * kTowerBumperDistance;
            targetY = tagPose.getY() + tagRightY * kTowerBumperDistance;
        }

        // Normalize heading to [-π, π] for continuous input controller
        targetHeading = normalizeAngle(targetHeading);

        // Seed controllers with current state — prevents derivative kick on start
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        rotController.reset(robotPose.getRotation().getRadians());

        xController.setGoal(targetX);
        yController.setGoal(targetY);
        rotController.setGoal(targetHeading);

        SmartDashboard.putString("AlignTower/Status",
            "Aligning (" + (robotOnLeft ? "LEFT side" : "RIGHT side") + ")...");
        SmartDashboard.putString("AlignTower/Side", robotOnLeft ? "LEFT" : "RIGHT");
    }

    @Override
    public void execute() {
        // If rejected on initialize(), do nothing — don't move the robot
        if (tooFar) return;

        Pose2d robotPose = drivetrain.getState().Pose;

        double xOut   = xController.calculate(robotPose.getX());
        double yOut   = yController.calculate(robotPose.getY());
        double rotOut = rotController.calculate(normalizeAngle(robotPose.getRotation().getRadians()));

        drivetrain.setControl(
            driveRequest
                .withVelocityX(xOut)
                .withVelocityY(yOut)
                .withRotationalRate(rotOut)
        );

        // ── Dashboard telemetry ───────────────────────────────────────────────
        boolean transAligned = xController.atGoal() && yController.atGoal();
        boolean rotAligned   = rotController.atGoal();

        SmartDashboard.putNumber("AlignTower/ErrorX_m",
            targetX - robotPose.getX());
        SmartDashboard.putNumber("AlignTower/ErrorY_m",
            targetY - robotPose.getY());
        SmartDashboard.putNumber("AlignTower/ErrorRot_deg",
            Math.toDegrees(targetHeading - normalizeAngle(robotPose.getRotation().getRadians())));

        if (transAligned && rotAligned) {
            // ✅ FULLY ALIGNED — display confirmation on Elastic
            SmartDashboard.putString("AlignTower/Status", "TOWER ALIGNED");
            SmartDashboard.putBoolean("AlignTower/Aligned", true);
        } else {
            SmartDashboard.putBoolean("AlignTower/Aligned", false);
            if (!rotAligned && !transAligned) {
                SmartDashboard.putString("AlignTower/Status", "Rotating + Driving...");
            } else if (!rotAligned) {
                SmartDashboard.putString("AlignTower/Status", "Rotating...");
            } else {
                SmartDashboard.putString("AlignTower/Status", "Driving...");
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when button is released or command is interrupted
        drivetrain.setControl(
            driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
        SmartDashboard.putString("AlignTower/Status", "—");
        SmartDashboard.putBoolean("AlignTower/Aligned", false);
    }

    @Override
    public boolean isFinished() {
        // Never self-terminates — runs until LT is released (whileTrue handles this)
        // Exception: if rejected due to distance, end immediately
        return tooFar;
    }

    // ── Helper: normalize angle to [-π, π] ───────────────────────────────────
    private double normalizeAngle(double radians) {
        while (radians >  Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}
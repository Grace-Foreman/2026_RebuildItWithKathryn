// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.overkillShoot;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.overkillShoot.ShootConstants;

/**
 * AlignToGoalCommand — rotates the robot to face the goal and pre-spins
 * the flywheel to the correct physics-calculated speed.
 *
 * Bound to: Operator LB (whileTrue)
 *
 * Behavior:
 *   - Continuously rotates toward the nearest goal AprilTag
 *   - Calculates required flywheel RPS from horizontal distance (projectile physics)
 *   - Spins the flywheel to that speed (does NOT formally require Shoot subsystem
 *     so that ShootCommand can override it cleanly when RB is pressed)
 *   - Displays "READY TO FIRE! (RB)" on SmartDashboard when aligned + up to speed
 *   - Runs until LB is released
 *
 * Based on FRC 3940's AimAtSpeakerCommand (2024) with 2026 game adaptations.
 */
public class AlignToGoalCommand extends Command {

    // ── Tunable constants ─────────────────────────────────────────────────────
    // Rotation PID — tune kP first, then add kD if oscillating
    private static final double kRotP = 0.05;  //0.8 // Matches 3940's kAimP
    private static final double kRotI = 0.0;
    private static final double kRotD = 0.1;
    // Max angular velocity and acceleration for the profile
    private static final double kMaxAngVel  = 1.0 * Math.PI; // rad/s
    private static final double kMaxAngAccel = 1.0 * Math.PI; // rad/s²

    // Rotation tolerance — how close is "aligned" (radians)
    private static final double kRotTolerance = Math.toRadians(1);
    private static final double kShooterOffset = Math.toRadians(-4);

    // Flywheel "up to speed" tolerance (RPS)
    //private static final double kSpeedTolerance = 1.0;
    // ─────────────────────────────────────────────────────────────────────────

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem         visionSubsystem;
    //private final Shoot                   shootSubsystem; // NOT in addRequirements — ShootCommand overrides

   // private final PIDController rotationController = new PIDController(kRotD, kMaxAngVel, kMaxAngAccel);
    
     private final ProfiledPIDController rotationController = new ProfiledPIDController(
        kRotP, kRotI, kRotD,
        new TrapezoidProfile.Constraints(kMaxAngVel, kMaxAngAccel)
     );

    // CTRE swerve request — field-centric drive with PID-controlled rotation
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AlignToGoalCommand(CommandSwerveDrivetrain drivetrain,
                               VisionSubsystem visionSubsystem
                               /*, Shoot shootSubsystem*/) {
        this.drivetrain      = drivetrain;
        this.visionSubsystem = visionSubsystem;
        //this.shootSubsystem  = shootSubsystem;

        // Only require the drivetrain — Shoot is NOT required here so that
        // ShootCommand (RB) can take over the subsystem without interrupting
        // the drive alignment when the operator presses RB.
        addRequirements(drivetrain);

        // Wrap rotation error across 0 to 2π — prevents spinning the wrong way
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(kRotTolerance);
    }

    @Override
    public void initialize() {
        // Seed the controller with current robot angle so it doesn't snap
        rotationController.reset(
            drivetrain.getState().Pose.getRotation().getRadians(), 0
        );
        SmartDashboard.putString("AlignGoal/Status", "Aligning...");
    }

    @Override
public void execute() {
    boolean isBlue = visionSubsystem.isBlue();

    // ── 1. Get goal target pose ───────────────────────────────────────────
    Pose2d goalPose  = visionSubsystem.getClosestGoalTarget(isBlue);
    Pose2d robotPose = drivetrain.getState().Pose;

    // ── 2. Calculate angle from robot to goal ─────────────────────────────
    double dx = goalPose.getX() - robotPose.getX();
    double dy = goalPose.getY() - robotPose.getY();
    double targetAngle = Math.atan2(dy, dx);

    // FIX 1: DO NOT normalize to [0, 2π] — keep in [-π, π] to match
    // enableContinuousInput(-PI, PI). The offset is small enough not to matter.
    targetAngle += kShooterOffset;
    // Clamp back into [-π, π] after offset just in case
    targetAngle = MathUtil.angleModulus(targetAngle);

    // ── 3. PID: calculate rotation output ─────────────────────────────────
    rotationController.setGoal(targetAngle);
    double currentAngle = robotPose.getRotation().getRadians();

    // FIX 2: Let the ProfiledPIDController handle tolerance — do NOT manually
    // zero the output. Bypassing it desynchronizes the internal motion profile
    // and causes the drift hop when you re-enter the active zone.
    double rotOutput = rotationController.calculate(currentAngle);

    // Optional: small deadband to prevent buzzing at steady state
    if (Math.abs(rotOutput) < 0.01) rotOutput = 0.0;

    // ── 4. Apply drive command ────────────────────────────────────────────
    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rotOutput)
    );

    boolean aligned = rotationController.atGoal();
    SmartDashboard.putString("AlignGoal/Status", aligned ? "Aligned" : "Rotating");
    SmartDashboard.putNumber("AlignGoal/TargetAngleDeg", Math.toDegrees(targetAngle));
    SmartDashboard.putNumber("AlignGoal/CurrentAngleDeg", Math.toDegrees(currentAngle));
    SmartDashboard.putNumber("AlignGoal/RotOutput", rotOutput);
}
    @Override
    public void end(boolean interrupted) {
     
        // Stop drivetrain rotation
        drivetrain.setControl(
            driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
        // Let flywheel coast — ShootCommand will take over if RB is still held
        //shootSubsystem.coastTopShoot();
        
    }

    @Override
    public boolean isFinished() {
        
        return false; // Runs until LB is released
    }
}
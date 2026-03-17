// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AlignToGoalCommand;
import frc.robot.commands.AlignToTowerCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Telemetry;

/**
 * RobotContainer for Team 6875 — 2026 season.
 *
 * Built from a clean CTRE Tuner X generated swerve base.
 *
 * ── Control Scheme ──────────────────────────────────────────────────────────
 *
 *  Driver Controller (port 0):
 *    Left Stick        → Translate (field-centric)
 *    Right Stick X     → Rotate
 *    Start             → Zero gyro angle (align to alliance wall)
 *
 *  Operator Controller (port 1):
 *    LB                → AlignToGoalCommand (rotate to goal + spin up flywheel)
 *    LT                → AlignToTowerCommand (drive + rotate to tower, hold alignment)
 *    RB                → ShootCommand (fire — feeder engages once flywheel is at speed)
 *    Y                 → Intake (placeholder — wire to IntakeCommand when ready)
 *    A                 → Climb Level 3 (placeholder — wire to climb command when ready)
 *
 * NOTE: If drivers prefer AlignToGoal on their controller, move the LB binding
 *       from operator to driver by changing `operator.leftBumper()` → `driver.leftBumper()`
 *       and adjusting addRequirements in AlignToGoalCommand if needed.
 *
 * ── TODO items ──────────────────────────────────────────────────────────────
 *  [ ] Set Flywheel and Feeder CAN IDs in Shoot.java ShootConstants
 *  [ ] Wire operator Y → IntakeCommand (replace placeholder InstantCommand below)
 *  [ ] Wire operator A → Climb Level 3 command (replace placeholder below)
 *  [ ] Update AprilTagFields in VisionSubsystem once 2026 field layout is released
 *  [ ] Tune vision std devs, PID gains, and kTowerBumperDistance on robot
 */
public class RobotContainer {

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystems
    // ─────────────────────────────────────────────────────────────────────────
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem visionSubsystem    = new VisionSubsystem(drivetrain);
    private final Shoot           shootSubsystem     = new Shoot();

    // ─────────────────────────────────────────────────────────────────────────
    // Controllers
    // ─────────────────────────────────────────────────────────────────────────
    private final CommandXboxController driver   = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // ─────────────────────────────────────────────────────────────────────────
    // Swerve drive settings (from CTRE Tuner X generated template)
    // ─────────────────────────────────────────────────────────────────────────
    private final double kMaxSpeed       = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Slew-rate limited joystick inputs for smoother driving
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxSpeed * 0.1)
        .withRotationalDeadband(kMaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake   = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt    point   = new SwerveRequest.PointWheelsAt();

    private final Telemetry telemetry = new Telemetry(kMaxSpeed);

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────
    public RobotContainer() {
        configureBindings();
        putDashboard();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Bindings
    // ─────────────────────────────────────────────────────────────────────────
    private void configureBindings() {

        // ── Driver: default swerve drive ─────────────────────────────────────
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-driver.getLeftY() * kMaxSpeed)
                    .withVelocityY(-driver.getLeftX() * kMaxSpeed)
                    .withRotationalRate(-driver.getRightX() * kMaxAngularRate)
            )
        );

        // Driver A → brake (X-lock wheels)
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Driver B → point wheels (steer only, no translation)
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(
                new Rotation2d(-driver.getLeftY(), -driver.getLeftX())
            )
        ));

        // Driver Start → re-zero gyro toward alliance wall
        driver.start().onTrue(new InstantCommand(() -> {
            Rotation2d resetAngle = Rotation2d.fromDegrees(0);
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                resetAngle = Rotation2d.fromDegrees(180);
            }
            Translation2d currentPos = drivetrain.getState().Pose.getTranslation();
            drivetrain.resetPose(new Pose2d(currentPos, resetAngle));
        }).ignoringDisable(true).withName("Zero Gyro"));

        // Driver SysId bindings (for initial characterization runs)
        // Back + Y/X = dynamic | Start + Y/X = quasistatic
        // These are inherited from the CTRE Tuner X generated template.

        // ── Operator: game bindings ───────────────────────────────────────────

        // Operator LB → align to goal (rotate robot + pre-spin flywheel)
        // NOTE: Move to driver.leftBumper() if drivers prefer to control this
        operator.leftBumper().whileTrue(
            new AlignToGoalCommand(drivetrain, visionSubsystem, shootSubsystem)
        );

        // Operator LT → align to tower (drive to tower + hold side-specific heading)
        // Automatically detects left/right side of tower.
        // Rejects if robot is >2m away (tunable in VisionConstants.kMaxTowerAlignDistance).
        operator.leftTrigger().whileTrue(
            new AlignToTowerCommand(drivetrain, visionSubsystem)
        );

        // Operator RB → fire shooter (flywheel + feeder once up to speed)
        operator.rightBumper().whileTrue(
            new ShootCommand(shootSubsystem)
        );

        // Operator Y → intake
        // TODO: Replace this placeholder with your real IntakeCommand
        operator.y().whileTrue(
            new InstantCommand(() -> {
                // TODO: new IntakeCommand(intakeSubsystem)
            }).withName("IntakePlaceholder")
        );

        // Operator A → climb level 3 (one-button sequence)
        // TODO: Replace this placeholder with your climb sequence command
        // e.g.: operator.a().onTrue(new ClimbLevel3Command(climbSubsystem));
        operator.a().onTrue(
            new InstantCommand(() -> {
                // TODO: Wire to climb command when code is ready
            }).withName("ClimbL3Placeholder")
        );

        // Drivetrain telemetry registration
        drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Dashboard setup
    // ─────────────────────────────────────────────────────────────────────────
    private void putDashboard() {
        // Zero angle button on dashboard (useful when no controller is plugged in)
        SmartDashboard.putData("Zero Gyro", new InstantCommand(() -> {
            Rotation2d resetAngle = Rotation2d.fromDegrees(0);
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                resetAngle = Rotation2d.fromDegrees(180);
            }
            Translation2d currentPos = drivetrain.getState().Pose.getTranslation();
            drivetrain.resetPose(new Pose2d(currentPos, resetAngle));
        }).ignoringDisable(true));

        // Vision status
        SmartDashboard.putString("AlignGoal/Status",  "—");
        SmartDashboard.putString("AlignTower/Status", "—");
        SmartDashboard.putBoolean("AlignTower/Aligned", false);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Auto
    // ─────────────────────────────────────────────────────────────────────────
    public Command getAutonomousCommand() {
        // TODO: Add autonomous routines using PathPlanner or command sequences
        return Commands.none();
    }
}
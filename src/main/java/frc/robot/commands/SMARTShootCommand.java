// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.SmartShoot;
import frc.robot.subsystems.SmartShoot.ShootConstants;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ShootCommand — spins flywheel to interpolated zone speed, latches feeder once up to speed.
 *
 * Bound to: Operator B (whileTrue)
 *
 * ── Tuning workflow ───────────────────────────────────────────────────────────
 *  1. Deploy and stand at 1.5 m from goal.
 *  2. Hold B. Watch Shoot/TargetRPS and Shoot/CurrentRPS on Elastic.
 *     Confirm CurrentRPS tracks TargetRPS — if not, tune kV in Shoot.java.
 *  3. Once tracking, check if shot lands. Adjust kZone1RPS in Shoot.java.
 *  4. Repeat at 3.0 m (kZone2RPS), 4.5 m (kZone3RPS), 6.0 m (kZone4RPS).
 *  5. Test intermediate distances — interpolation handles the rest.
 */
public class SmartShootCommand extends Command {

    private final SmartShoot                   smartShoot;
    private final VisionSubsystem         visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;

    // How close to target RPS before feeder engages — TUNABLE
    // Too tight → feeder never turns on. Too loose → feeder fires before wheel is ready.
    private static final double RPS_TOLERANCE = 2.0;

    // Latch: feeder turns on once and stays on until end()
    // Prevents rapid on/off jitter that shakes the robot
    private boolean feederEngaged = false;

    public SmartShootCommand(VisionSubsystem visionSubsystem,
                        SmartShoot smartShoot,
                        CommandSwerveDrivetrain drivetrain) {
        this.visionSubsystem = visionSubsystem;
        this.smartShoot  = smartShoot;
        this.drivetrain      = drivetrain;
        addRequirements(smartShoot);
    }

    @Override
    public void initialize() {
        feederEngaged = false;
        // Start flywheel immediately so it begins ramping before execute() runs
        smartShoot.setFlywheelRPS(smartShoot.calculateRPS(getDistanceToGoal()));
        SmartDashboard.putString("Shoot/Status", "Spinning up...");
    }

    @Override
    public void execute() {
        // ── 1. Live distance ──────────────────────────────────────────────────
        double distanceMeters = getDistanceToGoal();

        // ── 2. Interpolated target RPS from zone table ────────────────────────
        // calculateRPS() handles all zone logic and out-of-range clamping
        double targetRPS = smartShoot.calculateRPS(distanceMeters);

        // ── 3. Always update flywheel to latest interpolated value ────────────
        smartShoot.setFlywheelRPS(targetRPS);

        // ── 4. Latch feeder on once flywheel reaches target ───────────────────
        if (!feederEngaged && smartShoot.isUpToSpeed(targetRPS, RPS_TOLERANCE)) {
            feederEngaged = true;
        }

        if (feederEngaged) {
            // Feeder runs at same RPS as flywheel
            smartShoot.setFeederRPS(targetRPS);
        }

        // ── 5. Dashboard — use to verify zone boundaries and PID tracking ──────
        boolean inRange = distanceMeters >= ShootConstants.kZone1Distance
                       && distanceMeters <= ShootConstants.kZone4Distance;

        SmartDashboard.putNumber("Shoot/TargetRPS",  targetRPS);
        SmartDashboard.putNumber("Shoot/CurrentRPS", smartShoot.getFlywheelRPS());
        SmartDashboard.putNumber("Shoot/DistanceM",  distanceMeters);
        SmartDashboard.putBoolean("Shoot/AtSpeed",   feederEngaged);
        SmartDashboard.putString("Shoot/Zone",       getZoneLabel(distanceMeters));
        SmartDashboard.putString("Shoot/Status",
            feederEngaged ? "FIRING"
            : !inRange    ? (distanceMeters < ShootConstants.kZone1Distance ? "TOO CLOSE" : "TOO FAR")
                          : "Spinning up...");
    }

    @Override
    public void end(boolean interrupted) {
        smartShoot.stopFeeder();   // Feeder brakes immediately
        smartShoot.stopFlywheel(); // Flywheel coasts down
        SmartDashboard.putString("Shoot/Status",   "—");
        SmartDashboard.putBoolean("Shoot/AtSpeed", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until B is released
    }

    // ── Helper: straight-line distance to nearest goal tag ───────────────────
    private double getDistanceToGoal() {
        Pose2d goal  = visionSubsystem.getClosestGoalTarget(visionSubsystem.isBlue());
        Pose2d robot = drivetrain.getState().Pose;
        double dx = goal.getX() - robot.getX();
        double dy = goal.getY() - robot.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    // ── Helper: human-readable zone label for dashboard ──────────────────────
    private String getZoneLabel(double d) {
        if (d <= ShootConstants.kZone1Distance) return "Zone 1 (≤1.5m)";
        if (d <= ShootConstants.kZone2Distance) return "Zone 1→2";
        if (d <= ShootConstants.kZone3Distance) return "Zone 2→3";
        if (d <= ShootConstants.kZone4Distance) return "Zone 3→4";
        return "Zone 4 (>4.5m)";
    }
}
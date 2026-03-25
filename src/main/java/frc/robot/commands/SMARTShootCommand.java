// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SMARTShoot;
import frc.robot.subsystems.SMARTShoot.ShootConstants;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ShootCommand — calculates flywheel RPS from distance every loop,
 * spins up flywheel (CAN 16), then latches feeder (CAN 15) on at the same speed.
 *
 * Bound to: Operator B (whileTrue)
 *
 * To tune:
 *   Watch Shoot/TargetRPS and Shoot/CurrentRPS on Elastic.
 *   If those match but ball misses: adjust kEfficiency in Shoot.java.
 *   If PID doesn't track well: tune kV in Shoot.java (most important).
 */
public class SMARTShootCommand extends Command {

    private final SMARTShoot                   smartShoot;
    private final VisionSubsystem         visionSubsystem;
    private final CommandSwerveDrivetrain drivetrain;

    // How close to target RPS before feeder engages — TUNABLE
    private static final double RPS_TOLERANCE = 2.0;

    // Latch: feeder turns on once and stays on until end()
    private boolean feederEngaged = false;

    public SMARTShootCommand(VisionSubsystem visionSubsystem,
                        SMARTShoot smartShoot,
                        CommandSwerveDrivetrain drivetrain) {
        this.visionSubsystem = visionSubsystem;
        this.smartShoot  = smartShoot;
        this.drivetrain      = drivetrain;
        addRequirements(smartShoot);
    }

    @Override
    public void initialize() {
        feederEngaged = false;
        // Start flywheel immediately so it ramps up before execute() runs
        smartShoot.setFlywheelRPS(smartShoot.calculateRPS(getDistanceToGoal()));
        SmartDashboard.putString("Shoot/Status", "Spinning up...");
    }

    @Override
    public void execute() {
        double distanceMeters = getDistanceToGoal();
        boolean inRange = distanceMeters >= ShootConstants.kMinRangeMeters
                       && distanceMeters <= ShootConstants.kMaxRangeMeters;

        // Recalculate every loop — flywheel adjusts as robot moves
        double targetRPS = inRange
            ? smartShoot.calculateRPS(distanceMeters)
            : ShootConstants.kMinRPS;

        // Flywheel always tracks physics value
        smartShoot.setFlywheelRPS(targetRPS);

        // Latch feeder on once flywheel hits target RPS
        if (!feederEngaged && smartShoot.isUpToSpeed(targetRPS, RPS_TOLERANCE)) {
            feederEngaged = true;
        }

        if (feederEngaged) {
            // Feeder runs at same RPS as flywheel for consistent feed rate
            smartShoot.setFeederRPS(10);
        }

        SmartDashboard.putNumber("Shoot/TargetRPS",  targetRPS);
        SmartDashboard.putNumber("Shoot/CurrentRPS", smartShoot.getFlywheelRPS());
        SmartDashboard.putNumber("Shoot/DistanceM",  distanceMeters);
        SmartDashboard.putBoolean("Shoot/AtSpeed",   feederEngaged);
        SmartDashboard.putString("Shoot/Status",
            !inRange
                ? (distanceMeters < ShootConstants.kMinRangeMeters ? "⚠ TOO CLOSE" : "⚠ TOO FAR")
                : feederEngaged ? "FIRING" : "Spinning up...");
    }

    @Override
    public void end(boolean interrupted) {
        smartShoot.stopFeeder();   // Feeder brakes first
        smartShoot.stopFlywheel(); // Flywheel coasts
        SmartDashboard.putString("Shoot/Status",   "—");
        SmartDashboard.putBoolean("Shoot/AtSpeed", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until B is released
    }

    private double getDistanceToGoal() {
        Pose2d goal  = visionSubsystem.getClosestGoalTarget(visionSubsystem.isBlue());
        Pose2d robot = drivetrain.getState().Pose;
        double dx = goal.getX() - robot.getX();
        double dy = goal.getY() - robot.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}


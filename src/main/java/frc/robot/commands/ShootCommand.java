// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoot;

/**
 * ShootCommand — fires the shooter.
 *
 * Bound to: Operator RB (whileTrue)
 *
 * Behavior:
 *   1. On initialize: starts spinning the flywheel (top motor)
 *   2. On execute:    once flywheel reaches TARGET_VELOCITY_RPS,
 *                     engages the feeder (bottom motor) to push fuel through
 *   3. On end:        feeder stops FIRST (hard brake), then flywheel coasts
 *                     down naturally — protects motors, reduces wear
 *
 * This command never self-terminates (isFinished = false).
 * It runs until the operator releases RB.
 *
 * NOTE: If AlignToGoalCommand is running simultaneously on LB,
 * it will be spinning the flywheel as a pre-spin. ShootCommand will
 * take over the Shoot subsystem when RB is pressed (it requires Shoot).
 * When RB is released, AlignToGoalCommand resumes if LB is still held.
 */
public class ShootCommand extends Command {

    private final Shoot shootSubsystem;

    // ── Tunable constants — adjust to match your robot ───────────────────────
    private static final double TOP_MOTOR_VOLTAGE    = 8.0;  // Volts for flywheel motor
    private static final double BOTTOM_MOTOR_VOLTAGE = 8.0;  // Volts for feeder motor
    private static final double TARGET_VELOCITY_RPS  = 96.416; // Target flywheel speed (RPS)
    private static final double TOLERANCE_RPS        = 2.0;  // Acceptable speed error (RPS)
    // ─────────────────────────────────────────────────────────────────────────

    public ShootCommand(Shoot shootSubsystem) {
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        // Start spinning the flywheel immediately so it reaches speed ASAP
        shootSubsystem.setVoltageTopShoot(TOP_MOTOR_VOLTAGE);
    }

    @Override
    public void execute() {
        // Keep flywheel running
        shootSubsystem.setVoltageTopShoot(TOP_MOTOR_VOLTAGE);

        // Only engage feeder once flywheel is up to speed
        // This prevents wasted fuel from going through before the wheel is ready
        if (shootSubsystem.isUpToSpeed(TARGET_VELOCITY_RPS, TOLERANCE_RPS)) {
            shootSubsystem.setVoltageBottomShoot(BOTTOM_MOTOR_VOLTAGE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // ── ORDER MATTERS ──────────────────────────────────────────────────
        // 1. Stop feeder FIRST — hard brake so no extra fuel is fed through
        shootSubsystem.stopBottomShoot();

        // 2. Let flywheel coast naturally — don't hard-brake a spinning wheel,
        //    it stresses the gearbox and motor.
        //    Flywheel motor neutral mode is set to Coast in Shoot.java.
        shootSubsystem.coastTopShoot();
        // ──────────────────────────────────────────────────────────────────
    }

    @Override
    public boolean isFinished() {
        // Runs until operator releases RB (whileTrue binding handles this)
        return false;
    }
}
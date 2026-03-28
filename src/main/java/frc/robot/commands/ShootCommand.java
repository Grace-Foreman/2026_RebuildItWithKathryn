// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shoot;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command {

    private final Shoot shootSubsystem;

    // Tunable constants - adjust these to match your robot
    private static final double TOP_MOTOR_VOLTAGE    = 8.5;  // Volts to run top motor
    private static final double BOTTOM_MOTOR_VOLTAGE = 8.5;  // Volts to run bottom motor
    private static final double TARGET_VELOCITY_RPS  = 60.0; // Target speed in rotations/sec
    private static final double TOLERANCE_RPS        = 2.0;  // Acceptable speed tolerance

    public ShootCommand(Shoot shoot) {
        this.shootSubsystem = shoot;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        // Start spinning the top motor immediately
        shootSubsystem.setVoltageTopShoot(TOP_MOTOR_VOLTAGE);
    }

    @Override
    public void execute() {
        // Once the top motor reaches target speed, spin the bottom motor too
        if (shootSubsystem.isUpToSpeed(TARGET_VELOCITY_RPS, TOLERANCE_RPS)) {
            shootSubsystem.setVoltageBottomShoot(BOTTOM_MOTOR_VOLTAGE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both motors when the command ends
        shootSubsystem.stopTopShoot();
        shootSubsystem.stopBottomShoot();
    }

    @Override
    public boolean isFinished() {
        // Command runs until the button is released (whileTrue binding)
        return false;
    }
}
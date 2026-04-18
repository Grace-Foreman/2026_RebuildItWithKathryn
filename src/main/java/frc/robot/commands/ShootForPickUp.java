// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shoot;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootForPickUp extends Command {
  
    private final Shoot shootSubsystem;
     private final Intake intakeSubsystem;

    Timer shootTimer = new Timer();
    Timer intakeTimer = new Timer();

    // Tunable constants - adjust these to match your robot
    private static final double TOP_MOTOR_VOLTAGE    = 7.0;  // Volts to run top motor
    private static final double BOTTOM_MOTOR_VOLTAGE = 7.0;  // Volts to run bottom motor
    private static final double TARGET_VELOCITY_RPS  = 20.0; // Target speed in rotations/sec
    private static final double TOLERANCE_RPS        = 4.0;  // Acceptable speed tolerance

    public ShootForPickUp(Shoot shootSubsystem, Intake intakeSubsystem) {
        this.shootSubsystem = shootSubsystem;
         this.intakeSubsystem = intakeSubsystem;
       
      

       addRequirements(shootSubsystem, intakeSubsystem);
    }@Override
    public void initialize() {
        // Start spinning the top motor immediately
        System.out.println("auahukwdoawd");
        shootSubsystem.setVoltageTopShoot(TOP_MOTOR_VOLTAGE);
        shootTimer.reset();
        shootTimer.start();
        intakeTimer.reset();
        intakeTimer.start();
    }

    @Override
    public void execute() {
        // Once the top motor reaches target speed, spin the bottom motor too
        if (shootSubsystem.isUpToSpeed(TARGET_VELOCITY_RPS, TOLERANCE_RPS)) {
            shootSubsystem.setVoltageBottomShoot(BOTTOM_MOTOR_VOLTAGE);
        }
    
      if (intakeTimer.hasElapsed(0)) {
        intakeSubsystem.setVoltage(-6.0);

      }
        if (intakeTimer.hasElapsed(0.5)){
            intakeSubsystem.stop();
       }

       if (intakeTimer.hasElapsed(1)) {
         intakeTimer.stop();
         intakeTimer.reset();
         intakeTimer.start();
       }

    }

    @Override
    public void end(boolean interrupted) {
        // Stop both motors when the command ends
        shootSubsystem.stopTopShoot();
        shootSubsystem.stopBottomShoot();
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
       return shootTimer.hasElapsed(5);
    }
}
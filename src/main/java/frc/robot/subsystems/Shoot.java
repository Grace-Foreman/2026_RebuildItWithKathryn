// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends SubsystemBase {
  private final TalonFX topShootMotor;
  private final TalonFX bottomShootMotor;
  private final VelocityVoltage velocityVoltage;
    
    // PID Constants - tune these for your mechanism
    private static final double kP = 6.0;  // Proportional gain
    private static final double kI = 0.0;   // Integral gain
    private static final double kD = 0.0;   // Derivative gain
    private static final double kS = 0.1;   // Static feedforward
    private static final double kV = 1.69;   // Velocity feedforward 1.69
    
    // Motion constraints
    private static final double MAX_VELOCITY = 85.0;      // rotations per second
    private static final double MAX_ACCELERATION = 170.; // rotations per second^2
    private static final double MAX_JERK = 200.0;        // rotations per second^3
    
    // Gear ratio: motor rotations per mechanism rotation
    private static final double GEAR_RATIO = 15.34;
  /** Creates a new Shoot. */
  public Shoot(int topMotorCanId, int bottomMotorCanId) {
     topShootMotor = new TalonFX(topMotorCanId);
     bottomShootMotor = new TalonFX(bottomMotorCanId);
     velocityVoltage = new VelocityVoltage(0).withSlot(0);
        
        configureMotor();
  }
private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Configure PID gains
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        
        // Configure motion magic for smooth profiled motion
        config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = MAX_JERK;
        
        // Configure motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
     // Set to true if motor is inverted
        
        // Configure current limits (optional but recommended)
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
       // config.CurrentLimits.SupplyTimeLimit = 0.1;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Apply configuration
        topShootMotor.getConfigurator().apply(config);
        bottomShootMotor.getConfigurator().apply(config);
        

    }
  
    public AngularVelocity  getMotorSpeedRPS(){
       return topShootMotor.getVelocity().getValue();
    }

    public boolean isUpToSpeed(double targetVelocityRPS, double toleranceRPS) {
      return topShootMotor.getVelocity().isNear(targetVelocityRPS, toleranceRPS);
    }

     public void setVoltageTopShoot(double voltage) {
        topShootMotor.setVoltage(voltage);
    }
    /**
     * Stop the motor
     */
    public void stopTopShoot() {
        topShootMotor.stopMotor();
    }    

    public void setVoltageBottomShoot(double voltage) {
        bottomShootMotor.setVoltage(voltage);
    }
    /**
     * Stop the motor
     */
    public void stopBottomShoot() {
        bottomShootMotor.stopMotor();
    }    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

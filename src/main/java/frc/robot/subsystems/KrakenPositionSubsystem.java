package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenPositionSubsystem extends SubsystemBase {
    private final TalonFX krakenMotor;
    private final PositionVoltage positionControl;
    
    // PID Constants - tune these for your mechanism
    private static final double kP = 6.0;  // Proportional gain
    private static final double kI = 0.0;   // Integral gain
    private static final double kD = 0.0;   // Derivative gain
    private static final double kS = 0.1;   // Static feedforward
    private static final double kV = 1.69;   // Velocity feedforward 1.69
    
    // Motion constraints
    private static final double MAX_VELOCITY = 2.0;      // rotations per second   Max speed
    private static final double MAX_ACCELERATION = 2.0; // rotations per second^2  How long it takes to get to make speed max velocity/ max accelerations = how long it take to get up ot make speed
    private static final double MAX_JERK = 200.0;        // rotations per second^3
    
    // Gear ratio: motor rotations per mechanism rotation
    private static final double GEAR_RATIO = 15.34;
    
    public KrakenPositionSubsystem(int canId) {
        krakenMotor = new TalonFX(canId);
        positionControl = new PositionVoltage(0).withSlot(0);
        
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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
     // Set to true if motor is inverted
        
        // Configure current limits (optional but recommended)
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
       // config.CurrentLimits.SupplyTimeLimit = 0.1;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Apply configuration
        krakenMotor.getConfigurator().apply(config);
        
        // Reset encoder position to 0
        krakenMotor.setPosition(0);
    }
    
    /**
     * Set the target position for the motor
     * @param rotations Target position in rotations
     */
    public void setPosition(double rotations) {
        krakenMotor.setControl(positionControl.withPosition(rotations * GEAR_RATIO));
    }
    
    /**
     * Get the current position of the motor
     * @return Current position in rotations
     */
    public double getPosition() {
        return krakenMotor.getPosition().getValueAsDouble() / GEAR_RATIO;
    }
    
    /**
     * Get the target position
     * @return Target position in rotations
     */
    public double getTargetPosition() {
        return krakenMotor.getClosedLoopReference().getValueAsDouble() / GEAR_RATIO;
    }
    
    /**
     * Check if the motor is at the target position
     * @param tolerance Tolerance in rotations
     * @return True if within tolerance
     */
    public boolean atTarget(double tolerance) {
        return Math.abs(getPosition() - getTargetPosition()) < tolerance;
    }
    
    /**
     * Reset the encoder position to 0
     */
    public void resetEncoder() {
        krakenMotor.setPosition(0);
    }
    
    /**
     * Set the encoder position to a specific value
     * @param rotations Position to set in rotations
     */
    public void setEncoderPosition(double rotations) {
        krakenMotor.setPosition(rotations * GEAR_RATIO);
    }
    
    /**
     * Stop the motor
     */
    public void stop() {
        krakenMotor.stopMotor();
    }
    
    /**
     * Command to move to a specific position
     * @param targetRotations Target position in rotations
     * @return Command that moves to the target position
     */
    public Command moveToPosition(double targetRotations) {
        return runOnce(() -> setPosition(targetRotations))
            .andThen(run(() -> {}))
            .until(() -> atTarget(1))
            .withName("MoveToPosition_" + targetRotations);
    }
    
    /**
     * Command to move to a position and hold it
     * @param targetRotations Target position in rotations
     * @return Command that moves to and holds the position
     */
    public Command moveToAndHold(double targetRotations) {
        return run(() -> setPosition(targetRotations))
            .withName("MoveToAndHold_" + targetRotations);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add telemetry here
    }
}


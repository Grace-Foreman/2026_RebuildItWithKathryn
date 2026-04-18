package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX krakenMotor;
    private final VelocityVoltage velocityVoltage;
   
   private final TalonFX intakeMotor = new TalonFX(15, "rio");
       // Use the correct Phoenix 6 unit types instead of <Double>
    private final StatusSignal<Current>  currentSignal   = intakeMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> velocitySignal  = intakeMotor.getVelocity();
    private final StatusSignal<Double>   dutyCycleSignal = intakeMotor.getDutyCycle(); // duty cycle stays as Double

    private static final double STALL_CURRENT_AMPS = 40.0;
    private static final double STALL_VELOCITY_RPS = 1.0;
    private static final double STALL_MIN_OUTPUT   = 0.1;

    public boolean isStalling() {
        double current   = currentSignal.getValueAsDouble();
        double velocity  = Math.abs(velocitySignal.getValueAsDouble());
        double dutyCycle = Math.abs(dutyCycleSignal.getValueAsDouble());

        return dutyCycle > STALL_MIN_OUTPUT
            && current   > STALL_CURRENT_AMPS
            && velocity  < STALL_VELOCITY_RPS;
    }
    
    // PID Constants - tune these for your mechanism
    private static final double kP = 6.0;  // Proportional gain
    private static final double kI = 0.0;   // Integral gain
    private static final double kD = 0.0;   // Derivative gain
    private static final double kS = 0.1;   // Static feedforward
    private static final double kV = 1.69;   // Velocity feedforward 1.69
    
    // Motion constraints
    private static final double MAX_VELOCITY = 4.0;      // rotations per second
    private static final double MAX_ACCELERATION = 20.0; // rotations per second^2
    private static final double MAX_JERK = 200.0;        // rotations per second^3
    
    // Gear ratio: motor rotations per mechanism rotation
    private static final double GEAR_RATIO = 15.34;
    

    
    public Intake(int canId) {
        krakenMotor = new TalonFX(canId);
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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
     // Set to true if motor is inverted
        
        // Configure current limits (optional but recommended)
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimit = 60.0;
       // config.CurrentLimits.SupplyTimeLimit = 0.1;
        // config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Apply configuration
        krakenMotor.getConfigurator().apply(config);
        
    }
    
    /**
     * Set the target position for the motor
     * @param rotations Target position in rotations
     */
    public void setVoltage(double voltage) {
        krakenMotor.setVoltage(voltage);
    }
    /**
     * Stop the motor
     */
    public void stop() {
        krakenMotor.stopMotor();
        
    }    
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add telemetry here
        StatusSignal.refreshAll(currentSignal, velocitySignal, dutyCycleSignal);

        SmartDashboard.putBoolean("Intake Stalling", isStalling());
        SmartDashboard.putNumber("Intake Current", currentSignal.getValueAsDouble());
        SmartDashboard.putNumber("Intake Velocity", velocitySignal.getValueAsDouble());
    }
}

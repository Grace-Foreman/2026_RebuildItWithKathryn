package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final Intake subsystem;
    private final double tolerance;
    private final double voltage;
    
    /**
     * Command to move the Kraken motor to a specific position
     * @param subsystem The RunSubsystem to use
     * @param voltage Target position in rotations
     * @param tolerance Position tolerance in rotations
     */
    public IntakeCommand(Intake subsystem, double voltage, double tolerance) {
        this.subsystem = subsystem;
        this.voltage = voltage;
        this.tolerance = tolerance;
        
        addRequirements(subsystem);
    }
    
    /**
     * Command with default tolerance of 0.1 rotations
     */
    public IntakeCommand(Intake subsystem, double voltage) {
        this(subsystem, voltage, 0);
    }
    
    @Override
    public void initialize() {
        subsystem.setVoltage(voltage);
    }
    
    @Override
    public void execute() {
        // Position control is handled by the motor controller
        // No need to update anything here
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        // If not interrupted, motor will hold position
    }
}

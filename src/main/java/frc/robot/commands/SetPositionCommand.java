package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenPositionSubsystem;

public class SetPositionCommand extends Command {
    private final KrakenPositionSubsystem subsystem;
    private final double targetPosition;
    private final double tolerance;
    
    /**
     * Command to move the Kraken motor to a specific position
     * @param subsystem The KrakenPositionSubsystem to use
     * @param targetPosition Target position in rotations
     * @param tolerance Position tolerance in rotations
     */
    public SetPositionCommand(KrakenPositionSubsystem subsystem, double targetPosition, double tolerance) {
        this.subsystem = subsystem;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
        
        addRequirements(subsystem);
    }
    
    /**
     * Command with default tolerance of 0.1 rotations
     */
    public SetPositionCommand(KrakenPositionSubsystem subsystem, double targetPosition) {
        this(subsystem, targetPosition, 0);
    }
    
    @Override
    public void initialize() {
        subsystem.setPosition(targetPosition);
    }
    
    @Override
    public void execute() {
        // Position control is handled by the motor controller
        // No need to update anything here
    }
    
    @Override
    public boolean isFinished() {
        return subsystem.atTarget(tolerance);
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            subsystem.stop();
        }
        // If not interrupted, motor will hold position
    }
}

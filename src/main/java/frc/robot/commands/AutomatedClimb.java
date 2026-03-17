package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.KrakenPositionSubsystem;

public class AutomatedClimb extends SequentialCommandGroup {
    
    /**
     * Automated sequence that cycles through positions 3 times
     * Pattern: Position 1 → Home → Position 2 → Home → Position 3 → Home → Position 4 → Home
     * Repeats 3 times
     * 
     * @param subsystem The KrakenPositionSubsystem to use
     * @param position1 First position value
     * @param position2 Second position value
     * @param position3 Third position value
     * @param homePosition Home position value
     */
    public AutomatedClimb(
            KrakenPositionSubsystem subsystem,
            double position1,
            double position2,
            double position3,
            double homePosition) {
        
        this(subsystem, position1, position2, position3,homePosition, 0.5, 3.0);
    }
    
    /**
     * Automated sequence with custom tolerance and timeout
     * 
     * @param subsystem The KrakenPositionSubsystem to use
     * @param position1 First position value
     * @param position2 Second position value
     * @param position3 Third position value
     * @param homePosition Home position value
     * @param tolerance Position tolerance in rotations
     * @param moveTimeout Timeout for each movement in seconds
     */
    public AutomatedClimb(
            KrakenPositionSubsystem subsystem,
            double position1,
            double position2,
            double position3,
            double homePosition,
            double tolerance,
            double moveTimeout) {
        
        addCommands(
            // Cycle 1
            new SetPositionCommand(subsystem, position1, moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            new SetPositionCommand(subsystem, position2, moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            new SetPositionCommand(subsystem, position3,  moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            Commands.print("=== Cycle 1 Complete ==="),
            
            // Cycle 2
            new SetPositionCommand(subsystem, position2, moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            new SetPositionCommand(subsystem, position3, moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            Commands.print("=== Cycle 2 Complete ==="),
            
            // Cycle 3
            new SetPositionCommand(subsystem, position2, moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            new SetPositionCommand(subsystem, position3, moveTimeout),
            new SetPositionCommand(subsystem, homePosition, moveTimeout),
            Commands.print("=== ALL CYCLES COMPLETE ===")
        );
        
        setName("AutomatedSequence_3Cycles");
    }
}
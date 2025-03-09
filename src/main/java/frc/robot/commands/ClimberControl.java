package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

/**
 * Contains all control mechanisms for the intake.
 */
public class ClimberControl extends Command {

    private Climber climber;

    /**
     * Initializes a new intake controller command base.

     * @param subsystem - The climber subsystem to run off of.
     */
    public ClimberControl(Climber subsystem) {
        this.climber = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {}

    public Command sendIt = Commands.runOnce(() -> {
        climber.climb(-1);
    });
        
    public Command pinch = Commands.runOnce(() -> {
        climber.climb(ClimberConstants.climberThrottle);
    });

    public Command release = Commands.runOnce(() -> {
        climber.unClimb(ClimberConstants.climberThrottle);
    });

    public Command stopClimber = Commands.runOnce(() -> {
        climber.stopClimber();

    });
}
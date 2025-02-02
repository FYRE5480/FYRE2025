package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

/**
 * Contains all control mechanisms for the intake.
 */
public class IntakeControl extends Command {

    //PLACEHOLDER VALUES
    /**
     * Rotates the intake at the given speed for the given ammount of seconds.

     * @param seconds - the amount of seconds to run the intake
     * @param time - the amount of time the intake has been running 
     * @param speed - the speed to rotate the actuation
     * @param wheelSpeed - the speed to rotate flywheels
     */
    //PLACEHOLDER VALUES
    int speed = -1234567890;
    int wheelSpeed = -1234567890;

    private Intake intake;

    /**
     * Initializes a new intake controller command base.

     * @param subsystem - The Intake subsystem to run off of.
     */
    public IntakeControl(Intake subsystem) {
        this.intake = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {

    }

    public Command intakeUp = Commands.runOnce(() -> {
        intake.goUp(speed);
    });
    public Command intakeDown = Commands.runOnce(() -> {
        intake.goDown(speed);
    });
    public Command intakeCoral = Commands.runOnce(() -> {
        intake.intakeCoral(wheelSpeed);;
    });
}

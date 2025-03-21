package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;

/** The elevator lifting functionality for our arm. */
public class ArmControl extends Command {

    // The elevator subsystem
    private Arm arm;

    /**
     * Creates a new elevator command.

     * @param subsystem - the elevator subsystem
     */
    public ArmControl(Arm subsystem) {
        this.arm = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {}

    /**
     * Sends the arm, to top using PID.
     */
    public Command goToCoral = Commands.runOnce(() -> {
        arm.goToCoral();
    });

    public Command goToLowerAlgae = Commands.runOnce(() -> {
        arm.goToLowerAlgae();
    });

    public Command goToUpperAlgae = Commands.runOnce(() -> {
        arm.goToUpperAlgae();
    });

    public Command goToBarge = Commands.runOnce(() -> {
        arm.goToBarge();
    });

    /**
     * Sends the arm to the bottom using PID.
     */
    public Command goToBottom = Commands.runOnce(() -> {
        arm.goToBottom();
    });

    public Command runMotorForwards = Commands.runOnce(() -> {
        arm.runMotorForward();
    });

    public Command runMotorBackward = Commands.runOnce(() -> {
        arm.runMotorBackward();
    });

    /**
     * Stops motors using the PID stop and the motor stop.
     */
    public Command stopMotors = Commands.runOnce(() -> {
        arm.stopMotor();
    });

    public Command toggleManualOverride = Commands.runOnce(() -> {
        arm.toggleManualOverride();
    });


    @Override
    public boolean isFinished() {
        return arm.atGoal();
    }
}
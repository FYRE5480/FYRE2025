package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;

/** The elevator lifting functionality for our arm. */
public class ElevatorLift extends Command {

    // The elevator subsystem
    private Elevator elevator;

    /**
     * Creates a new elevator command.

     * @param subsystem - the elevator subsystem
     */
    public ElevatorLift(Elevator subsystem) {
        this.elevator = subsystem;
        addRequirements(subsystem);
    }

    /**
     * Called repeatedly when a command is scheduled.
     */
    @Override
    public void execute() {}

    /**
     * Sends the elevator to top using PID.
     */
    public Command goToTop = Commands.runOnce(() -> {
        elevator.goToTop();
    });

    public Command goToMid = Commands.runOnce(() -> {
        elevator.goToMid();
    });

    /**
     * Sends the elevator to the bottom using PID.
     */
    public Command goToBottom = Commands.runOnce(() -> {
        elevator.goToBottom();
    });

    /**
     * Runs the elevator up.
     */
    public Command runMotorForward = Commands.runOnce(() -> {
        elevator.runMotorForward();
    });

    /**
     * runs the elevator down.
     */
    public Command runMotorReverse = Commands.runOnce(() -> {
        elevator.runMotorBackward();
    });

    public Command toggleManualOverride = Commands.runOnce(() -> {
        elevator.toggleManualOverride();
    });

    /**
     * Stops motors using the PID stop and the motor stop.
     */
    public Command stopMotors = Commands.runOnce(() -> {
        elevator.stopMotor();
    });
}
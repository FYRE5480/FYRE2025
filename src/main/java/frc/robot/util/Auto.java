package frc.robot.util;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ClawControl;
import frc.robot.commands.ElevatorLift;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

/** Simple wrapper class to load routines. */
public class Auto {
    
    Swerve swerve;
    ElevatorLift elevator;
    ClawControl claw;
    ArmControl arm;

    AutoFactory autoFactory;

    /**
     * Constructs a new Auto object with a swerve object to use for path control.

     * @param swerve - the swerve object to be used
     */
    public Auto(Swerve swerve, ElevatorLift elevator, ClawControl claw, ArmControl arm) {

        this.swerve = swerve;
        this.elevator = elevator;
        this.claw = claw;
        this.arm = arm;

        autoFactory = new AutoFactory(
			swerve::getPose,
			swerve::resetOdometry, 
            swerve::followTrajectory, 
            false, 
            swerve
        );
    }

    // runs the dummy minifigure8 routine
    public AutoRoutine real() {
        AutoRoutine figure8 = autoFactory.newRoutine("simpleRight");

        AutoTrajectory figure8Trajectory = figure8.trajectory("SimpleRight");

        figure8.active().onTrue(
            Commands.sequence(
                figure8Trajectory.resetOdometry(),
                figure8Trajectory.cmd()
            )
        );

        figure8Trajectory.atTime("SlowWheels")
            .onTrue(claw.slowHold);

        figure8Trajectory.atTime("goToScore")
            .onTrue(arm.goToCoral)
            .onTrue(elevator.goToTop);

        figure8Trajectory.atTime("release")
            .onTrue(claw.output);
        
        figure8Trajectory.atTime("algae")
            .onTrue(elevator.goToBottom)
            .onTrue(arm.goToUpperAlgae);

        figure8Trajectory.atTime("down")
            .onTrue(elevator.goToBottom)
            .onTrue(arm.goToBottom);

    

        return figure8;
    }

    
    // runs the dummy minifigure8 routine
    public AutoRoutine miniFigure8() {
        AutoRoutine figure8 = autoFactory.newRoutine("simpleMini8");

        AutoTrajectory figure8Trajectory = figure8.trajectory("SimpleMini8");

        figure8.active().onTrue(
            Commands.sequence(
                figure8Trajectory.resetOdometry(),
                figure8Trajectory.cmd()
            )
        );

        return figure8;
    }

    // runs the dummy figure8 routine
    public AutoRoutine figure8() {
        AutoRoutine figure8 = autoFactory.newRoutine("figure8");

        AutoTrajectory figure8Trajectory = figure8.trajectory("Figure8");

        figure8.active().onTrue(
            Commands.sequence(
                figure8Trajectory.resetOdometry(),
                figure8Trajectory.cmd()
            )
        );

        return figure8;
    }

    // routines need to be loaded at start time - not runtime before we're about to use it
    // loading routines takes time so don't do it when enabled
    public AutoRoutine dummyRoutine() {
        AutoRoutine exampleRoutine = autoFactory.newRoutine("example");

        AutoTrajectory exampleTrajectory = exampleRoutine.trajectory("exampleTrajectory");
        AutoTrajectory exampleTrajectory2 = exampleRoutine.trajectory("exampleTrajectory2");

        exampleRoutine.active().onTrue(
            Commands.sequence(
                exampleTrajectory.resetOdometry(),
                exampleTrajectory.cmd()
            )
        );

        // we can define auto routines PROGRAMATICALLY!!!

        // exampleTrajectory.atTime("exampleEvent").onTrue(/* some command here */);
        // exampleTrajectory.done().onTrue(exampleTrajectory2.cmd());

        // exampleTrajectory2.active().whileTrue(/* some other command here */);
        // exampleTrajectory2.done().onTrue(/* another command here */);

        return exampleRoutine;
    }
}

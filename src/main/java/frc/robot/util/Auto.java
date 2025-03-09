package frc.robot.util;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ClawControl;
import frc.robot.commands.ElevatorLift;
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

    public AutoRoutine fromLeft() {
        DataLogManager.log("Starting Auto Routine: fromLeft");

        AutoRoutine fromLeft = autoFactory.newRoutine("fromLeft");

        AutoTrajectory midFromLeft = fromLeft.trajectory("midFromLeft");
        AutoTrajectory midToScore = fromLeft.trajectory("midToScore");
        AutoTrajectory scoreToCoral = fromLeft.trajectory("leftScoreToAlgae");
        AutoTrajectory alg2 = fromLeft.trajectory("alg2");

        // update current pose of robot to starting point of first trajectory
        if (midFromLeft.getInitialPose().isEmpty()) {
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Autonomous", "Could not get initial pose from trajectory!"));
        } else {
            swerve.setPose(midFromLeft.getInitialPose().get());
        }

        fromLeft.active().onTrue(
            Commands.sequence(
                midFromLeft.resetOdometry(),
                midFromLeft.cmd()
            )
        );

        midFromLeft.done().onTrue(midToScore.cmd());
        midToScore.done().onTrue(scoreToCoral.cmd());
        scoreToCoral.done().onTrue(alg2.cmd());


        alg2.atTime("spit")
            .onTrue(claw.output);

        alg2.atTime("suck")
            .onTrue(claw.intake);

        alg2.atTime("up")
            .onTrue(elevator.goToBottom)
            .onTrue(arm.goToUpperAlgae);

        // this code is fine
        alg2.atTime("asdf")
            .onTrue(arm.goToBottom);

        alg2.atTime("asdf")
            .onTrue(elevator.goToBottom);

        // the following code causes arm.goToBottom to run many, many times
        // alg2.atTime("asdf")
        //     .onTrue(elevator.goToBottom)
        //     .onTrue(arm.goToBottom);

        midFromLeft.atTime("slowSuck")
            .onTrue(claw.slowHold);

        midToScore.atTime("goToScore")
            .onTrue(elevator.goToTop)
            .onTrue(arm.goToCoral);

        midToScore.atTime("spit")
            .onTrue(claw.output);

        midToScore.atTime("goDown")
            .onTrue(elevator.goToMid)
            .onTrue(arm.goToLowerAlgae);

        scoreToCoral.atTime("suck")
            .onTrue(claw.intake);

        scoreToCoral.atTime("goToAlgae")
            .onTrue(elevator.goToMid)
            .onTrue(arm.goToLowerAlgae);

        scoreToCoral.atTime("lower")
            .onTrue(elevator.goToBottom)
            .onTrue(arm.goToBottom);
    
        scoreToCoral.atTime("spit")
            .onTrue(claw.output);
    
        return fromLeft;
    }

    public AutoRoutine fromRight() {
        DataLogManager.log("Starting Auto Routine: fromRight");

        AutoRoutine fromRight = autoFactory.newRoutine("fromRight");

        AutoTrajectory midFromRight = fromRight.trajectory("midFromRight");
        AutoTrajectory midToScore = fromRight.trajectory("midToScore");
        AutoTrajectory scoreToCoral = fromRight.trajectory("scoreToCoral");

        // update current pose of robot to starting point of first trajectory
        if (midFromRight.getInitialPose().isEmpty()) {
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Autonomous", "Could not get initial pose from trajectory!"));
        } else {
            swerve.setPose(midFromRight.getInitialPose().get());
        }

        fromRight.active().onTrue(
            Commands.sequence(
                midFromRight.resetOdometry(),
                midFromRight.cmd()
            )
        );

        midFromRight.done().onTrue(midToScore.cmd());
        midToScore.done().onTrue(scoreToCoral.cmd());

        midToScore.atTime("goToScore")
            .onTrue(elevator.goToTop)
            .onTrue(arm.goToCoral);

        midToScore.atTime("spit")
            .onTrue(claw.output);

        scoreToCoral.atTime("goToAlgae")
            .onTrue(elevator.goToMid)
            .onTrue(arm.goToLowerAlgae);

        scoreToCoral.atTime("lower")
            .onTrue(elevator.goToBottom)
            .onTrue(arm.goToBottom);
    
        scoreToCoral.atTime("spit")
            .onTrue(claw.output);
    
        return fromRight;
    }

    public AutoRoutine fromMid() {
        DataLogManager.log("Starting Auto Routine: fromMid");

        AutoRoutine fromMid = autoFactory.newRoutine("fromMid");

        AutoTrajectory midFromMid = fromMid.trajectory("midFromMid");
        AutoTrajectory midToScore = fromMid.trajectory("midToScore");
        AutoTrajectory scoreToCoral = fromMid.trajectory("scoreToCoral");

        // update current pose of robot to starting point of first trajectory
        if (midFromMid.getInitialPose().isEmpty()) {
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Autonomous", "Could not get initial pose from trajectory!"));
        } else {
            swerve.setPose(midFromMid.getInitialPose().get());
        }

        fromMid.active().onTrue(
            Commands.sequence(
                midFromMid.resetOdometry(),
                midFromMid.cmd()
            )
        );

        midToScore.atTime("goToScore")
            .onTrue(elevator.goToMid)
            .onTrue(arm.goToLowerAlgae);

        midToScore.atTime("spit")
            .onTrue(claw.output);

        scoreToCoral.atTime("goToAlgae")
            .onTrue(elevator.goToMid)
            .onTrue(arm.goToLowerAlgae);

        scoreToCoral.atTime("lower")
            .onTrue(elevator.goToBottom)
            .onTrue(arm.goToBottom);
    
        scoreToCoral.atTime("spit")
            .onTrue(claw.output);
    
        return fromMid;
    }
}

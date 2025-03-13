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

        AutoTrajectory startLeft = fromLeft.trajectory("startLeft");
        AutoTrajectory leftToScore = fromLeft.trajectory("leftToScore");
        AutoTrajectory leftScoreToAlgae = fromLeft.trajectory("leftScoreToAlgae");
        AutoTrajectory leftAlg2 = fromLeft.trajectory("leftAlg2");

        // update current pose of robot to starting point of first trajectory
        if (startLeft.getInitialPose().isEmpty()) {
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Autonomous", "Could not get initial pose from trajectory!"));
        } else {
            swerve.setPose(startLeft.getInitialPose().get());
        }

        fromLeft.active().onTrue(
            Commands.sequence(
                startLeft.resetOdometry(),
                startLeft.cmd()
            )
        );

        startLeft.done().onTrue(leftToScore.cmd());
        leftToScore.done().onTrue(leftScoreToAlgae.cmd());
        leftScoreToAlgae.done().onTrue(leftAlg2.cmd());


        leftAlg2.atTime("spit")
            .onTrue(claw.output);

        leftAlg2.atTime("suck")
            .onTrue(claw.intake);

        leftAlg2.atTime("up")
            .onTrue(elevator.goToBottom)
            .onTrue(arm.goToUpperAlgae);


        leftAlg2.atTime("down")
            .onTrue(elevator.goToBottom)
            .onTrue(claw.stopWheels);

        leftAlg2.atTime("barge")
            .onTrue(elevator.goToTop)
            .onTrue(arm.goToBarge);

        // the following code causes arm.goToBottom to run many, many times
        // leftAlg2.atTime("asdf")
        //     .onTrue(elevator.goToBottom)
        //     .onTrue(arm.goToBottom);

        startLeft.atTime("slowSuck")
            .onTrue(claw.slowHold);

        leftToScore.atTime("goToScore")
            .onTrue(elevator.goToTop)
            .onTrue(arm.goToCoral);

        leftToScore.atTime("spit")
            .onTrue(claw.output);

        leftToScore.atTime("goDown")
            .onTrue(elevator.goToMid)
            .onTrue(arm.goToLowerAlgae);

        leftScoreToAlgae.atTime("suck")
            .onTrue(claw.intake);

        leftScoreToAlgae.atTime("up")
            .onTrue(elevator.goToTop)
            .onTrue(arm.goToBarge);
    
        leftScoreToAlgae.atTime("spit")
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
        AutoTrajectory midScoreToAlgae = fromMid.trajectory("midScoreToAlgae");

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

        midFromMid.atTime("slowHold")
            .onTrue(claw.slowHold);

        midToScore.atTime("goToScore")
            .onTrue(elevator.goToTop)
            .onTrue(arm.goToCoral);

        midToScore.atTime("spit")
            .onTrue(claw.output);

        midToScore.atTime("goToAlgae")
            .onTrue(elevator.goToBottom)
            .onTrue(arm.goToUpperAlgae);

    
        midScoreToAlgae.atTime("suck")
            .onTrue(claw.intake);


        midScoreToAlgae.atTime("goToBarge")
            .onTrue(elevator.goToTop)
            .onTrue(arm.goToBarge);
        
        midScoreToAlgae.atTime("spit")
            .onTrue(claw.output);
    
        return fromMid;
    }
}

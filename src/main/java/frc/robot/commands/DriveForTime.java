package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;



/**
 * asfd.
 */
public class DriveForTime extends Command {

    Swerve swerve;

    Timer time;

    double timeToDrive = 3;

    public DriveForTime(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(this.swerve);

        time = new Timer();
    }


    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    @Override
    public void execute() {
        swerve.swerveDrive(
            new ChassisSpeeds(
                -1,
                0,
                0
            )
        );
    }

    @Override
    public boolean isFinished() {
        return time.get() > timeToDrive;
    }

    @Override
    public void end(boolean isInterrupted) {
        swerve.swerveDrive(
            new ChassisSpeeds(
                0,
                0,
                0
            )
        );
    }


    
}

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AlignWithReef extends Command {
    
    Vision vision;
    Swerve swerve;

    ChassisSpeeds currentSpeeds;

    public AlignWithReef(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;

        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // TODO: use the proper overload of this method
        // currentSpeeds = vision.getTagDrive(1);

        // swerve.swerveDrive(currentSpeeds);
    }

    @Override
    public boolean isFinished() {
        return 
            currentSpeeds.vxMetersPerSecond < 0.5 
            && currentSpeeds.vyMetersPerSecond < 0.5
            && currentSpeeds.omegaRadiansPerSecond < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.swerveDrive(
            new ChassisSpeeds(
                0, 0, 0
            )
        );
    }

}


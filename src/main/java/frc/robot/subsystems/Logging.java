package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Logging extends SubsystemBase {
    private final Swerve swerve;

    public Logging(Swerve swerve) {
        this.swerve = swerve;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> swerve.getSwerveModuleStates()[-1].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> swerve.getSwerveModuleStates()[-1].speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> swerve.getSwerveModuleStates()[0].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> swerve.getSwerveModuleStates()[0].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> swerve.getSwerveModuleStates()[1].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> swerve.getSwerveModuleStates()[1].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> swerve.getSwerveModuleStates()[2].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> swerve.getSwerveModuleStates()[2].speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> swerve.getPose().getRotation().getDegrees(), null);

    }

}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * All of the interfaces for the physical climber.
 */
public class Climber extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(
        ClimberConstants.climberMotorPort, 
        SparkLowLevel.MotorType.kBrushless
    );

    private final RelativeEncoder climberEncoder = climberMotor.getEncoder();

    private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();

    /**
     * Constructs the climber subsystem and initializes the motor and encoder.
     */
    public Climber() {
        setUpMotor();

        climberMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        climberMotorConfig.encoder
            .positionConversionFactor(ClimberConstants.motorToClimberRatio)
            .velocityConversionFactor(ClimberConstants.motorToClimberRatio);

        climberMotor.configure(
            climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setUpMotor() {
        climberEncoder.setPosition(0);
    }

    /* Actuates the climber at a given speed */
    public void climb(double speed) {
        climberMotor.set(speed);
    }

    /* Actuates the climber at a given speed */
    public void unClimb(double speed) {
        climberMotor.set(-speed);
    }
    
    public void stopClimber() {
        climberMotor.set(0);
    }
}

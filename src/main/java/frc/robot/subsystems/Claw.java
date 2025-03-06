package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * All of the interfaces for the physical claw.
 */
public class Claw extends SubsystemBase {
    private final SparkMax clawMotor = new SparkMax(
        ClawConstants.clawMotorPort, 
        SparkLowLevel.MotorType.kBrushed
    );

    private final RelativeEncoder clawEncoder = clawMotor.getEncoder();

    private final SparkMaxConfig clawConfig = new SparkMaxConfig();

    /**
     * Constructs the claw subsystem and initializes the motor and encoder.
     */
    public Claw() {
        setUpMotor();

        clawMotor.configure(
            clawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setUpMotor() {
        clawEncoder.setPosition(0);
    }

    /* Runs the intake wheels at the given speed.*/
    public void intake(double speed) {
        clawMotor.set(speed);
    }

    public void output(double speed) {
        clawMotor.set(-speed);
    }

    public void stopWheels() {
        clawMotor.set(0);
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * All of the interfaces for the physical intake.
 */
public class Intake extends SubsystemBase {

    private final SparkMax intakeWheels = new SparkMax(
        IntakeConstants.intakeWheelPort, 
        SparkLowLevel.MotorType.kBrushed
    );

    // The neo motor that will handle the intake actuation.
    private final SparkMax intakeActuation = new SparkMax(
        IntakeConstants.intakeActuationPort,
        SparkLowLevel.MotorType.kBrushed
    );

    private final Encoder intakeActuationEncoder = new Encoder(
        IntakeConstants.intakeEncoderA, IntakeConstants.intakeEncoderB);

    private final SparkMaxConfig intakeActuationConfig = new SparkMaxConfig();

    private final SparkMaxConfig intakeWheelsConfig = new SparkMaxConfig();

    /**
     * Constructs the elevator subsystem and inializes motors, controllers, and the like.
     */
    public Intake() {
        setUpMotors();
    }

    @Override
    public void periodic() {
        // System.out.println(intakeEncoderActuation.get());
    }

    private void setUpMotors() {
        intakeActuationEncoder.reset(); 

        intakeActuationEncoder
            .setDistancePerPulse(IntakeConstants.motorToIntakeRatio);

        intakeActuationConfig
            .idleMode(IdleMode.kBrake);

        intakeActuation.configure(
            intakeActuationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeWheels.configure(
            intakeWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Moves the intake up at a given speed.

     * @param speed - the speed at which to raise the intake
     */
    public void goUp(double speed) {
        intakeActuation.set(speed);
    }

    /**
     * Moves the intake down at a given speed.

     * @param speed - the speed at which to lower the intake
     */
    public void goDown(double speed) {
        intakeActuation.set(-speed);
    }

    /**
     * Runs the intake wheels at the given speed.

     * @param speed - the speed at which to run the wheels
     */
    public void intakeCoral(double speed) {
        intakeWheels.set(speed);
    }
}

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/**
 * All of the interfaces for the phyiscal arm.
 */
public class Arm extends SubsystemBase {
    // The CIM which will be the "leader" for the arm.
    private final SparkMax armMotor = new SparkMax(
        ArmConstants.armPort, 
        SparkLowLevel.MotorType.kBrushless
    );

    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    private final ArmFeedforward armFeedForward = new ArmFeedforward(
        ArmConstants.staticGain,
        ArmConstants.gravityGain,
        ArmConstants.velocityGain
    );

    private double coralStat = ArmConstants.coralState;
    private double lowerAlgaeStat = ArmConstants.lowerAlgaeState;
    private double upperAlgaeStat = ArmConstants.upperAlgaeState;
    private double bargeStat = ArmConstants.bargeState;

    private TrapezoidProfile.State coralState = new TrapezoidProfile.State(
        coralStat,
        0
    );

    private TrapezoidProfile.State lowerAlgaeState = new TrapezoidProfile.State(
        lowerAlgaeStat,
        0
    );

    private TrapezoidProfile.State upperAlgaeState = new TrapezoidProfile.State(
        upperAlgaeStat,
        0
    );

    private TrapezoidProfile.State bargeState = new TrapezoidProfile.State(
        bargeStat,
        0
    );
    
    //put extra defaults here
    
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(
        0,
        0.05
    );

    private boolean manualOverride = false;
    private boolean canMoveUp = true;
    private boolean canMoveDown = true;

    private final ProfiledPIDController armController = new ProfiledPIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(
            ArmConstants.maxVelocity,
            ArmConstants.maxAcceleration
        )
    );

    /**
     * Constructs a new Arm object and configures it's motors.
     */
    public Arm() {
        setUpMotors();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // check if the controller is not yet at it's goal and the manual override is not active
        if (!(manualOverride)) {    
            // set the setpoint to the controller
            armMotor.setVoltage(
                // armFeedForward.calculate(
                //     getEncoderDistance(),
                //     armController.getGoal().position)
                armController.calculate(
                    getEncoderDistance(),
                    armController.getGoal()
                )
            );
        } else {
            //armMotor.set(0);
        }

    }

    private void setUpMotors() {
        resetEncoders();

        armMotorConfig
            .inverted(true);

        armMotorConfig.encoder
            .positionConversionFactor(ArmConstants.motorToArmRatio)
            .velocityConversionFactor(ArmConstants.motorToArmRatio);

        armMotor.configure(
            armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void goToCoral() {
        armController.setGoal(coralState);
        manualOverride = false;
    }

    public void goToLowerAlgae() {
        armController.setGoal(lowerAlgaeState);
        manualOverride = false;
    }

    public void goToUpperAlgae() {
        armController.setGoal(upperAlgaeState);
        manualOverride = false;
    }

    public void goToBarge() {
        armController.setGoal(bargeState);
        manualOverride = false;
    }
    
    //sets the goal to the bottom state of the arm
    public void goToBottom() {
        armController.setGoal(bottomState);
        manualOverride = false;
    }
    
    /**
     * Runs the motor forward or "up" at the given constant speed.
     */
    public void runMotorForward() {
        if (canMoveUp) {
            armMotor.set(ArmConstants.armThrottle);
            manualOverride = true;
        } else {
            armMotor.set(0.1 * Math.cos(Units.rotationsToRadians(armEncoder.getPosition())));
        }
    }

    /**
     * Runs the motor backward or "down" at the given constant speed.
     */
    public void runMotorBackward() {
        if (canMoveDown) {
            armMotor.set(-ArmConstants.armThrottle);
            manualOverride = true;
        } else {
            armMotor.set(0.1 * Math.cos(Units.rotationsToRadians(armEncoder.getPosition())));
        }
    }

    public void stopMotor() {
        armMotor.stopMotor();
    }

    public double getEncoderDistance() {
        return armEncoder.getPosition();
    }

    private void resetEncoders() {
        armEncoder.setPosition(0.0);
    }


    /**
     * Toggles the manaul ovrride control of the arm.
     */
    public void toggleManualOverride() {
        manualOverride = !manualOverride;
    }

    /**
     * This will take in the output, and a set point,
     * and calculates the amount the motor needs to spin based on this input.
     */
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = armFeedForward.calculate(setpoint.position, setpoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        armMotor.setVoltage(output + feedforward);
    }

    /**
     * Method to be used by the PID controller under the hood,
     * this is not used in our code but it is essential to PID.
     * DO NOT DELETE THIS METHOD
     */
    protected double getMeasurement() {
        // possibly add an offset here? 
        return getEncoderDistance();
    }

    public boolean atGoal() {
        return armController.atGoal();
    }
}

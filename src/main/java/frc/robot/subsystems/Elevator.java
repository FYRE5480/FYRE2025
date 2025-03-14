// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorLiftConstants;

/**
 * The elevator subsystem to be used by any elevator commands. 
 */
public class Elevator extends SubsystemBase {

    // The NEO which will drive the elevator.
    private final SparkMax elevatorMotor = new SparkMax(
        ElevatorLiftConstants.elevatorMotorPort, 
        SparkLowLevel.MotorType.kBrushless
    );

    //The encoder on the elevator NEO 
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    private final SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
        ElevatorLiftConstants.staticGain,
        ElevatorLiftConstants.gravityGain,
        ElevatorLiftConstants.velocityGain
    );

    // the variable that will be used to calculate the maxiumum rotations to the top of the elevator
    private double rotationsToTop = ElevatorLiftConstants.rotationsToTop;
    
    // the trapezoid states for pre-defined elevator positions
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(
        0,
        0
    );
    
    private TrapezoidProfile.State semiTop = new TrapezoidProfile.State(
        ElevatorLiftConstants.rotationToSemiTop,
        0
    );

    private TrapezoidProfile.State middleState = new TrapezoidProfile.State(
        ElevatorLiftConstants.rotationsToMid,
        0
    );

    private TrapezoidProfile.State topState = new TrapezoidProfile.State(
        rotationsToTop,
        0
    );
    
    // variable to control if the driver needs to manually ovveride the elevator
    private boolean manualOverride = false;

    // variables to keep track of if elevator can move up and down
    private boolean canMoveUp = true;
    private boolean canMoveDown = true;

    // will control elevator speed and the motion profile
    private final ProfiledPIDController controller = new ProfiledPIDController(
        ElevatorLiftConstants.kP,
        ElevatorLiftConstants.kI,
        ElevatorLiftConstants.kD,
        new TrapezoidProfile.Constraints(
            ElevatorLiftConstants.maxVelocity,
            ElevatorLiftConstants.maxAcceleration
        )
    );


    /**
     * Constructs the elevator subsystem and configures the motors, controllers, and the like.
     */
    public Elevator() {
        setUpMotors();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // check if the controller is not yet at it's goal and the manual override is not active
        if (!(manualOverride)) { 
            // set the setpoint to the controller
            elevatorMotor.setVoltage(
                // elevatorFeedForward.calculate(
                //     controller.getGoal().velocity) 
                controller.calculate(
                    getEncoderDistances(),
                    controller.getGoal()
                )
            );
        } else {
            //elevatorMotor.set(0);
            
        }

        //System.out.println(getEncoderDistances());

    }

    //sets up the motors at the beginning of the program
    private void setUpMotors() {
        resetEncoders();
        
        controller.setTolerance(0.05);

        elevatorMotorConfig
            .inverted(true);

        elevatorMotorConfig.encoder
            .positionConversionFactor(ElevatorLiftConstants.motorToElevatorRatio)
            .velocityConversionFactor(ElevatorLiftConstants.motorToElevatorRatio);

        elevatorMotor.configure(
            elevatorMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kNoPersistParameters
        );
    }

    //sets the goal to the top state of the elevator
    public void goToTop() {
        controller.setGoal(topState);
        manualOverride = false;
    }

    public void goToSemiTop() {
        controller.setGoal(semiTop);
        manualOverride = false;
    }

    public void goToMid() {
        controller.setGoal(middleState);
        manualOverride = false;
    }
    
    //sets the goal to the bottom state of the elevator
    public void goToBottom() {
        controller.setGoal(bottomState);
        manualOverride = false;
    }

    public double getEncoderDistances() {
        return elevatorEncoder.getPosition();
    }

    private void resetEncoders() {
        elevatorEncoder.setPosition(0.0);
    }

    /**
     * Runs the motor forward or "up" at the given constant speed.
     */
    public void runMotorForward() {
        elevatorMotor.set(ElevatorLiftConstants.elvevatorThrottle);
        manualOverride = true;
    }

    /**
     * Runs the motor backward or "down" at the given constant speed.
     */
    public void runMotorBackward() {
        elevatorMotor.set(-ElevatorLiftConstants.elvevatorThrottle);
        manualOverride = true;
    }

    /**
     * Stops the elevator from moving.
     */
    public void stopMotor() {
        elevatorMotor.stopMotor();
    }

    /**
     * Returns the top state of the elevator.

     * @return topState - the top state of the elevator
     */
    public TrapezoidProfile.State getUpState() {
        return topState;
    }

    /**
     * Returns the bottoms state of the elevator.

     * @return bottomState - the bottom state of the elevator
     */
    public TrapezoidProfile.State getDownState() {
        return bottomState;

    }

    /**
     * Toggles the manaul ovrride control of the elevator.
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
        double feedforward = elevatorFeedForward.calculate(setpoint.position, setpoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        elevatorMotor.setVoltage(output + feedforward);
    }

    /**
     * Method to be used by the PID controller under the hood,
     * this is not used in our code but it is essential to PID.
     * DO NOT DELETE THIS METHOD
     */
    protected double getMeasurement() {
        // possibly add an offset here? 
        return getEncoderDistances();
    }
}
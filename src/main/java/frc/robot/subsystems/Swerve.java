package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.ControllerInput;
import frc.robot.util.ControllerInput.VisionStatus;
import frc.robot.util.SwerveModule;

/**
 * The physical subsystem that controls the drivetrain.
 */
public class Swerve extends SubsystemBase {
    private final ControllerInput controllerInput;

    private final Vision visionSystem; 
    private final AHRS gyroAhrs;

    private final SwerveModule[] swerveModules = new SwerveModule[4];

    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d currentPose;

    private final PIDController xController = new PIDController(
        DriveConstants.xyP, DriveConstants.xyI, DriveConstants.xyD);
    private final PIDController yController = new PIDController(
        DriveConstants.xyP, DriveConstants.xyI, DriveConstants.xyD);
    private final PIDController turnPID = new PIDController(
        DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD, DriveConstants.turnR);

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        DriveConstants.frontLeft, DriveConstants.frontRight,
        DriveConstants.backLeft, DriveConstants.backRight
    );

    private double startTime = Timer.getTimestamp();

    boolean setupComplete = false;

    /**
     * Constructs a swerve subsystem with the given controller and vision systems.

     * @param controller - the controller object that will be used to control the drive system
     * @param visionSystem - the vision system that will be used to control the drivetrain
     */
    public Swerve(ControllerInput controller, Vision visionSystem) {

        // assign constructor variables
        this.controllerInput = controller;
        this.visionSystem = visionSystem;

        // TODO: change this dynamically depending on the starting pose of the robot
        this.currentPose = new Pose2d(5.7, 2.1, new Rotation2d(0));

        // define the gyro
        gyroAhrs = new AHRS(NavXComType.kMXP_SPI);
        // reset the gyro
        gyroAhrs.reset();
        gyroAhrs.configureVelocity(
            false,
            false,
            false,
            true
        );

        // sets up the motors
        setupModules();
        
        // define pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics,
            gyroAhrs.getRotation2d(),
            getSwerveModulePositions(),
            currentPose 
        );

        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        currentPose = poseEstimator.updateWithTime(
            startTime - Timer.getTimestamp(), gyroAhrs.getRotation2d(), getSwerveModulePositions());

        // TODO add a timeout here or manual override to ensure setupComplete does not hold up entire robot

        // TODO maybe move this to constructor? or some other init function
        if (setupComplete) {
            if (!DriverStation.isAutonomousEnabled())
                swerveDrive(chooseDriveMode());
        } else setupCheck();
    }
     
    private ChassisSpeeds chooseDriveMode() {
        VisionStatus status = controllerInput.visionStatus();
        ChassisSpeeds speeds;

        switch (status) {
            case ALIGN_TAG: // lines the robot up with the tag
                speeds = visionSystem.alignTagSpeeds(0, null); 
                break;
            case LOCKON: // allows the robot to move freely by user input but remains facing the tag
                ChassisSpeeds controllerSpeeds = controllerInput.controllerChassisSpeeds(
                    turnPID, gyroAhrs.getRotation2d());
                ChassisSpeeds lockonSpeeds = visionSystem.lockonTagSpeeds(0, null);
                speeds = new ChassisSpeeds(
                    controllerSpeeds.vxMetersPerSecond,
                    controllerSpeeds.vyMetersPerSecond,
                    lockonSpeeds.omegaRadiansPerSecond
                );
                break;
            case GET_CORAL:
                speeds = visionSystem.getPieceDrive(0);
                break;
            default: // if all else fails - revert to drive controls
                speeds = controllerInput.controllerChassisSpeeds(turnPID, gyroAhrs.getRotation2d());
                break;
        }

        return speeds;
    }

    /**
     * Moves the robot using given ChassisSpeeds object.

     * @param chassisSpeeds - the chassis speed that the robot should take
     */
    public void swerveDrive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        boolean rotate = chassisSpeeds.vxMetersPerSecond != 0 
                        || chassisSpeeds.vyMetersPerSecond != 0 
                        || chassisSpeeds.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, DriveConstants.highDriveSpeed);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState targetState = moduleState[i];
            swerveModules[i].driveModule(targetState, rotate, controllerInput.nos(), controllerInput.throttle());
        }
    }

    /**
     * Get an array of SwerveModuleState objects for each swerve module in the drive.

     * @return swerveModuleStates - the array of module states
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            swerveModuleState[i] = swerveModules[i].getSwerveModuleState();
        }
        return swerveModuleState;
    }

    /**
     * Get an array of SwerveModulePosition objects for each swerve module in the drive.

     * @return swerveModulePositions - the array of module postions
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
        }
        return swerveModulePositions;
    }


    private void setupCheck() {
        visionSystem.clear();
        for (int i = 0; i < 4; i++) {
            if (swerveModules[i].setupCheck()) {
                return;
            }
        }

        setupComplete = true;
        System.out.println("----------\nSetup Complete!\n----------");
        setSwerveEncoders(0);
        for (int i = 0; i < 4; i++) swerveModules[i].setSwerveReference(0);
        try {TimeUnit.MILLISECONDS.sleep(20);} catch (InterruptedException e) {e.getStackTrace();} // TODO try removing this delay?
    }

    private void setupModules() {
        System.out.println("Setting up swerve modules");

        // if this needs to loop more than 4 times, something is very wrong
        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new SwerveModule(i);
        }
    }

    private void setSwerveEncoders(double position) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setSwerveEncoder(position);
        }
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {return swerveDriveKinematics;}

    public ChassisSpeeds getRobotState() {return swerveDriveKinematics.toChassisSpeeds(getSwerveModuleStates());}

    public Pose2d getPose() {return currentPose;} 

    public void resetGyro() {
        gyroAhrs.reset();
    }

    /** Prints the states of all 4 swerve modules. */
    public void printModuleStatus() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].printModuleStatus();
        }
    }

    /**
     * Sets the robot's odometry to match that of the given pose.

     * @param pose - the pose that the robot should assume
     */
    public void resetOdometry(Pose2d pose) {
        //resetEncoders();

        //gyroAhrs.reset();
        //gyroAhrs.setAngleAdjustment(pose.getRotation().getDegrees());

        currentPose = pose;
        poseEstimator.resetPose(pose);

    }

    // =============== AUTO STUFF ==================== //


    /**
     * Compiles and drives a ChassisSpeeds object from a given SwerveSample along the trajectory.

     * @param sample - the SwerveSample object that the robot should follow
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + turnPID.calculate(pose.getRotation().getRadians(), sample.heading),
            gyroAhrs.getRotation2d()
        );

        swerveDrive(speeds);
    }

}
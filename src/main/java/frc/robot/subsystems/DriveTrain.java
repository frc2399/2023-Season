// Maisie was HERE HAHAHHAHAHAHHAHA :D
// Shrividya was here :))
// Alice was here ;3
// Ethan was here
// John was here (\/)(;;)(\/)
// Clare was here!!!!!!!!!!!!!!
// Anna (the better one) was here :)
// Anna (the even better one) was here :)

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SimEncoder;
import frc.robot.util.SimGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {

    private static CANSparkMax leftFrontMotorController;
    private static CANSparkMax rightFrontMotorController;
    private static CANSparkMax leftBackMotorController;
    private static CANSparkMax rightBackMotorController;

    public static RelativeEncoder leftEncoder;
    public static RelativeEncoder rightEncoder;

    public AHRS ahrs;
    public static PIDController turnController;

    public static final double kP = 0;
    static final double kI = 0;
    static final double kD = 0;
    static final double kF = 0;

    public static double outputSpeed;

    // simulation
    private DifferentialDriveOdometry odometry;

    public SimEncoder leftEncoderSim;
    public SimEncoder rightEncoderSim;
    public SimGyro gyroSim;
    private DifferentialDrivetrainSim driveSim;
    public Field2d field = new Field2d();

    public DriveTrain() {

        leftFrontMotorController = new CANSparkMax(DriveConstants.LEFT_FRONT_DRIVE_CAN_ID, MotorType.kBrushless);
        rightFrontMotorController = new CANSparkMax(DriveConstants.RIGHT_FRONT_DRIVE_CAN_ID, MotorType.kBrushless);
        leftBackMotorController = new CANSparkMax(DriveConstants.LEFT_BACK_DRIVE_CAN_ID, MotorType.kBrushless);
        rightBackMotorController = new CANSparkMax(DriveConstants.RIGHT_BACK_DRIVE_CAN_ID, MotorType.kBrushless);

        leftFrontMotorController.restoreFactoryDefaults();
        rightFrontMotorController.restoreFactoryDefaults();
        leftBackMotorController.restoreFactoryDefaults();
        rightBackMotorController.restoreFactoryDefaults();

        // Set motors to brake mode
        leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Make wheels go in same direction
        leftFrontMotorController.setInverted(false);
        rightFrontMotorController.setInverted(true);

        // sets motor controllers following leaders
        leftBackMotorController.follow(leftFrontMotorController);
        rightBackMotorController.follow(rightFrontMotorController);

        // initialize motor encoder
        leftEncoder = leftFrontMotorController.getEncoder();
        rightEncoder = rightFrontMotorController.getEncoder();

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        leftEncoder.setPositionConversionFactor(DriveConstants.ENCODER_CALIBRATION_METERS);
        rightEncoder.setPositionConversionFactor(DriveConstants.ENCODER_CALIBRATION_METERS);

        // dividng by 60 to convert meters per miniute to meters per seconds
        leftEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_CALIBRATION_METERS / 60);
        rightEncoder.setVelocityConversionFactor(DriveConstants.ENCODER_CALIBRATION_METERS / 60);

        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();

        // this code is instantiating the simulated sensors and actuators when the robot is in simulation
        if (RobotBase.isSimulation()) {
            leftEncoderSim = new SimEncoder("Left Drive");
            rightEncoderSim = new SimEncoder("Right Drive");
            gyroSim = new SimGyro("NavX");
            // Create the simulation model of our drivetrain.
            driveSim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(3),       // 3 NEO motors on each side of the drivetrain.
                8,                       // 8:1 gearing reduction. for now
                6,                       // MOI of 6 kg m^2 (from CAD model). for now
                Units.lbsToKilograms(140), // The mass of the robot is 140 lbs (with battery) which is 63 kg
                Units.inchesToMeters(2.1), // The robot uses 2.1" radius wheels.
                Units.inchesToMeters(27.811), // The track width is 27.811 inches.

                // The standard deviations for measurement noise:
                // x and y:          0 m
                // heading:          0 rad
                // l and r velocity: 0  m/s
                // l and r position: 0 m
                VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
            );
        }
        odometry = new DifferentialDriveOdometry(getGyroAngle(), getLeftEncoderMeters(), getRightEncoderMeters(), new Pose2d(9, 6.5, new Rotation2d(3.14/2)));

        field = new Field2d();

        SmartDashboard.putData("Field", field);

        field.setRobotPose(new Pose2d(9, 6.5, new Rotation2d(3.14/2)));

    }

    @Override
    public void periodic() {

        odometry.update(
            // we want CCW positive, CW negative
            getGyroAngle(),
            getLeftEncoderMeters(),
            getRightEncoderMeters()
        );

        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("gyro angle", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("left encoder postion", getLeftEncoderMeters());
        SmartDashboard.putNumber("right encoder postion", getRightEncoderMeters());
        SmartDashboard.putNumber("left encoder velocity", getLeftEncoderMetersPerSecond());
        SmartDashboard.putNumber("right encoder veloicty", getRightEncoderMetersPerSecond());
        SmartDashboard.putNumber("odometry x", getPoseMeters().getX());
        SmartDashboard.putNumber("odometry y", getPoseMeters().getY());
        SmartDashboard.putNumber("odometry angle", getPoseMeters().getRotation().getDegrees());

    }

    @Override
    public void simulationPeriodic() {
         // This method will be called once per scheduler run when in simulation
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.

        driveSim.setInputs(
            leftFrontMotorController.get() * RobotController.getInputVoltage(),
            rightFrontMotorController.get() * RobotController.getInputVoltage()
        );
    
        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        driveSim.update(0.02);

        // Update all of our sensors.
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setSpeed(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setSpeed(driveSim.getRightVelocityMetersPerSecond());

        // we want CCW positive, CW negative
        gyroSim.setAngle(new Rotation2d(driveSim.getHeading().getRadians()));
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {
        leftFrontMotorController.set(leftSpeed);
        rightFrontMotorController.set(rightSpeed);
    }

    public void setMotorVoltage(double leftVolt, double rightVolt) {

        if (RobotBase.isSimulation()) {
            // Ethan hack: to convert voltage to percent
            // TODO: see if getInputVoltage is more accurate
            leftFrontMotorController.set(leftVolt / 12); //RobotController.getInputVoltage());
            rightFrontMotorController.set(rightVolt / 12); //RobotController.getInputVoltage());
        }
        else {
            leftFrontMotorController.setVoltage(leftVolt);
            rightFrontMotorController.setVoltage(rightVolt);
        }
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeedsMetersPerSecond() {        
        return new DifferentialDriveWheelSpeeds(getLeftEncoderMetersPerSecond(), getRightEncoderMetersPerSecond());
    }

    public Pose2d getPoseMeters(){
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftEncoderSim.setDistance(0);
        rightEncoderSim.setDistance(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistanceMeters() {
        return (getLeftEncoderMeters() + getRightEncoderMeters()) / 2.0;
    }

    
     /** Zeroes the heading of the robot. */
    public void zeroHeading() {
     gyroSim.setAngle(new Rotation2d());
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    // public double getTurnRate() {
    //     return -gyroSim.get;
    // }

    public Rotation2d getGyroAngle() {
        if (RobotBase.isSimulation()) {
            return gyroSim.getAngle();
        }
        else {
            // negative sign to make CCW positive
            return new Rotation2d(Units.degreesToRadians(-ahrs.getAngle()));
        }
    }

    public double getRightEncoderMeters() {
        if (RobotBase.isSimulation()) {
            return rightEncoderSim.getDistance();
        }
        else {
            return (rightEncoder.getPosition());
        }
    }

    public double getLeftEncoderMeters() {
        if (RobotBase.isSimulation()) {
            return leftEncoderSim.getDistance();
        }
        else {
            return (leftEncoder.getPosition());
        }
    }

    public double getRightEncoderMetersPerSecond() {
        if (RobotBase.isSimulation()) {
            return rightEncoderSim.getSpeed();
        }
        else {
            return (rightEncoder.getVelocity());
        }
    }

    public double getLeftEncoderMetersPerSecond() {
        if (RobotBase.isSimulation()) {
            return leftEncoderSim.getSpeed();
        }
        else {
            return leftEncoder.getVelocity();
        }
    }

}

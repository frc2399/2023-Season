// Maisie was HERE HAHAHHAHAHAHHAHA :D
// Shrividya was here :))
// Alice was here ;3
// Ethan was here
// John was here (\/)(;;)(\/)
// Clare was here!!!!!!!!!!!!!!
// Anna (the better one) was here :)
// Anna (the even better one) was here :)

package frc.robot.subsystems.drivetrain;

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

    private DriveIO driveIO;

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

        if (RobotBase.isSimulation()) {
            driveIO = new SimDrive();
        } else {
            driveIO = new RealDrive();
        }
 

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

        // runs sim periodic code in simDrive
        driveIO.updateForSim();

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

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {
        driveIO.setMotors(leftSpeed, rightSpeed);
    }

    public void setMotorVoltage(double leftVolt, double rightVolt) {
        driveIO.setMotorVoltage(leftVolt, rightVolt);
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
        return driveIO.getGyroAngle(); 
    }

    public double getRightEncoderMeters() {
        return driveIO.getRightEncoderMeters();
    }

    public double getLeftEncoderMeters() {
            return driveIO.getLeftEncoderMeters();
        }


    public double getRightEncoderMetersPerSecond() {
            return driveIO.getRightEncoderMetersPerSecond();
        }

    public double getLeftEncoderMetersPerSecond() {
            return driveIO.getLeftEncoderMetersPerSecond();
        }

}

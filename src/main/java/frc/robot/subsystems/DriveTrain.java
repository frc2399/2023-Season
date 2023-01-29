//Clare was here!!!!
//Herb was here !!!!
//Alison was here :)
//Maisie was here :))))))))))))))
//Rachel was here!!!! :DDDDDDD
//Alice was hwew :33333333333333
//Ethan was here (O-O)
//elle was hre

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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SimEncoder;
import frc.robot.util.SimGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {

    private static CANSparkMax leftFrontMotorController;
    private static CANSparkMax rightFrontMotorController;
    private static CANSparkMax leftMiddleMotorController;
    private static CANSparkMax rightMiddleMotorController;
    private static CANSparkMax leftBackMotorController;
    private static CANSparkMax rightBackMotorController;

    public static RelativeEncoder leftEncoder;
    public static RelativeEncoder rightEncoder;

    public static AHRS ahrs;
    public static PIDController turnController;

    public final double kP = 0;
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
        leftMiddleMotorController = new CANSparkMax(DriveConstants.LEFT_MIDDLE_DRIVE_CAN_ID, MotorType.kBrushless);
        rightMiddleMotorController = new CANSparkMax(DriveConstants.RIGHT_MIDDLE_DRIVE_CAN_ID, MotorType.kBrushless);
        leftBackMotorController = new CANSparkMax(DriveConstants.LEFT_BACK_DRIVE_CAN_ID, MotorType.kBrushless);
        rightBackMotorController = new CANSparkMax(DriveConstants.RIGHT_BACK_DRIVE_CAN_ID, MotorType.kBrushless);

        // Set motors to coast mode
        teleopInit();

        // Make wheels go in same direction
        leftFrontMotorController.setInverted(true);
        rightFrontMotorController.setInverted(false);

        // sets motor controllers following leaders
        leftMiddleMotorController.follow(leftFrontMotorController);
        rightMiddleMotorController.follow(rightFrontMotorController);
        leftBackMotorController.follow(leftFrontMotorController);
        rightBackMotorController.follow(rightFrontMotorController);

        turnController = new PIDController(kP, kI, kD);
        turnController.enableContinuousInput(-180.0f, 180.0f);

        // initialize motor encoder
        leftEncoder = leftFrontMotorController.getEncoder();
        rightEncoder = rightFrontMotorController.getEncoder();

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
        rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);

        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();

        //bruh which one
        //herb stop making fun of my spelling errors :(
        if (DriveConstants.IS_HIGH_SPEED) {

            DriveTrain.leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_SPEED_REVOLUTION_TO_INCH_CONVERSION);
            DriveTrain.rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_SPEED_REVOLUTION_TO_INCH_CONVERSION);
        }
        else {

            DriveTrain.leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
            DriveTrain.rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
        }

        // this code is instantiating the simulated sensors and actuators when the robot is in simulation
        if (RobotBase.isSimulation()) {

            leftEncoderSim = new SimEncoder("Left Drive");
            rightEncoderSim = new SimEncoder("Right Drive");
            gyroSim = new SimGyro("NavX");
            odometry = new DifferentialDriveOdometry(gyroSim.getAngle(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance(), new Pose2d(9, 6.5, new Rotation2d(3.14/2)));
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

            field = new Field2d();

            SmartDashboard.putData("Field", field);

            field.setRobotPose(new Pose2d(9, 6.5, new Rotation2d(3.14/2)));
        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
         // This method will be called once per scheduler run when in simulation
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.

        odometry.update(
            // we want CCW positive, CW negative
            new Rotation2d(gyroSim.getAngle().getRadians()),
            leftEncoderSim.getDistance(),
            rightEncoderSim.getDistance()
        );
        field.setRobotPose(odometry.getPoseMeters());

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

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoderSim.getSpeed(), rightEncoderSim.getSpeed());
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
        odometry.resetPosition(
            gyroSim.getAngle(), leftEncoderSim.getDistance(), rightEncoderSim.getDistance(), pose);
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
    public double getAverageEncoderDistance() {
        return (leftEncoderSim.getDistance() + rightEncoderSim.getDistance()) / 2.0;
    }

        /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public SimEncoder getLeftEncoder() {
        return leftEncoderSim;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public SimEncoder getRightEncoder() {
        return rightEncoderSim;
    }

     /** Zeroes the heading of the robot. */
    public void zeroHeading() {
     gyroSim.setAngle(new Rotation2d());
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyroSim.getAngle().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    // public double getTurnRate() {
    //     return -gyroSim.get;
    // }



    public double getLeftEncoderPosition() {
        if (RobotBase.isSimulation()) {
            return Units.metersToInches(leftEncoderSim.getDistance());
        }
        else {
            //gets position in inches
            return leftEncoder.getPosition();
        }
    }

    public double getRightEncoderPosition() {
        // gets position in inches
        if (RobotBase.isSimulation()) {
            return Units.metersToInches(rightEncoderSim.getDistance());
        }
        else {
            //gets position in inches
            return rightEncoder.getPosition();
        }
    }

    public double getAngle() {
        return ahrs.getAngle();
    }

    public static void teleopInit() {
        leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public static void autonomousInit() {
        leftFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFrontMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMiddleMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightBackMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

}

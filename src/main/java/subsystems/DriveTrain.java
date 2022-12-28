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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

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

    public DriveTrain() {

        leftFrontMotorController = new CANSparkMax(DriveConstants.LEFT_FRONT_MOTOR_CAN_ID, MotorType.kBrushless);
        rightFrontMotorController = new CANSparkMax(DriveConstants.RIGHT_FRONT_MOTOR_CAN_ID, MotorType.kBrushless);
        leftMiddleMotorController = new CANSparkMax(DriveConstants.LEFT_MIDDLE_MOTOR_CAN_ID, MotorType.kBrushless);
        rightMiddleMotorController = new CANSparkMax(DriveConstants.RIGHT_MIDDLE_MOTOR_CAN_ID, MotorType.kBrushless);
        leftBackMotorController = new CANSparkMax(DriveConstants.LEFT_BACK_MOTOR_CAN_ID, MotorType.kBrushless);
        rightBackMotorController = new CANSparkMax(DriveConstants.RIGHT_BACK_MOTOR_CAN_ID, MotorType.kBrushless);

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
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {
        leftFrontMotorController.set(leftSpeed);
        rightFrontMotorController.set(rightSpeed);
    }

    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        // gets position in inches
        return rightEncoder.getPosition();
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

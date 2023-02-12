package frc.robot.subsystems.drivetrain;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;


public class RealDrive implements DriveIO {
   // Basically copy over the entire rest of the drive subsystem that is "not sim"
   private static CANSparkMax leftFrontMotorController, rightFrontMotorController;
   private static CANSparkMax leftBackMotorController;
   private static CANSparkMax rightBackMotorController;
   public static RelativeEncoder leftEncoder, rightEncoder;
   public AHRS ahrs;


   public RealDrive() {
        rightFrontMotorController = new CANSparkMax(DriveConstants.RIGHT_FRONT_DRIVE_CAN_ID, MotorType.kBrushless);
        leftFrontMotorController = new CANSparkMax(DriveConstants.LEFT_FRONT_DRIVE_CAN_ID, MotorType.kBrushless);
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
   }
   

@Override
public double getRightEncoderMeters() {
    return (rightEncoder.getPosition());
}
public double getLeftEncoderMeters() {
    return (leftEncoder.getPosition());
}

@Override
public double getRightEncoderMetersPerSecond() {
    return rightEncoder.getVelocity();
}

@Override
public double getLeftEncoderMetersPerSecond() {
    return leftEncoder.getVelocity();
}

@Override
public Rotation2d getGyroAngle() {
    return new Rotation2d(Units.degreesToRadians(-ahrs.getAngle()));

}

@Override
public void setMotorVoltage(double leftVolt, double rightVolt) {
	leftFrontMotorController.setVoltage(leftVolt);
    rightFrontMotorController.setVoltage(rightVolt);
} 

public void setMotors(double leftSpeed, double rightSpeed) {
    leftFrontMotorController.set(leftSpeed);
    rightFrontMotorController.set(rightSpeed);
}

//explode!! :)
@Override
public void updateForReal()
{
    SmartDashboard.putNumber("LF Celsius", leftFrontMotorController.getMotorTemperature());
    SmartDashboard.putNumber("LB Celsius", leftBackMotorController.getMotorTemperature());
    SmartDashboard.putNumber("RF Celsius", rightFrontMotorController.getMotorTemperature());
    SmartDashboard.putNumber("RB Celsius", rightBackMotorController.getMotorTemperature());
}

@Override
public void updateForSim() {
    // TODO Auto-generated method stub
}
}

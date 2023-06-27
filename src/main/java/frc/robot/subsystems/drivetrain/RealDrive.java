package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.MotorUtil;
import frc.robot.util.NavX.AHRS;


public class RealDrive implements DriveIO {
   // Basically copy over the entire rest of the drive subsystem that is "not sim"
   private static CANSparkMax leftFrontMotorController, rightFrontMotorController, leftBackMotorController, rightBackMotorController;
   public static RelativeEncoder leftEncoder, rightEncoder;
   public static AHRS ahrs;


   public RealDrive() {
        
        rightFrontMotorController = MotorUtil.createSparkMAX(DriveConstants.RIGHT_FRONT_DRIVE_CAN_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, false, true, 0);
        leftFrontMotorController = MotorUtil.createSparkMAX(DriveConstants.LEFT_FRONT_DRIVE_CAN_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, true, 0);
        
        rightBackMotorController = MotorUtil.createSparkMAX(DriveConstants.RIGHT_BACK_DRIVE_CAN_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, 0);
        leftBackMotorController = MotorUtil.createSparkMAX(DriveConstants.LEFT_BACK_DRIVE_CAN_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, 0);

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


        ahrs = new AHRS(SPI.Port.kMXP, (byte) 66);
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

    leftSpeed = Math.max(Math.min(leftSpeed, 0.4), -0.4);

    rightSpeed = Math.max(Math.min(rightSpeed, 0.4), -0.4);

    leftFrontMotorController.set(leftSpeed);
    rightFrontMotorController.set(rightSpeed);
}

@Override
public double getGyroPitch() {
    return -ahrs.getPitch();
}


@Override
public void periodicUpdate() {
    //motor temps
    SmartDashboard.putNumber("drive/lf temp (C)", leftFrontMotorController.getMotorTemperature());
    SmartDashboard.putNumber("drive/lb temp (C)", leftBackMotorController.getMotorTemperature());
    SmartDashboard.putNumber("drive/rf temp (C)", rightFrontMotorController.getMotorTemperature());
    SmartDashboard.putNumber("drive/rb temp (C)", rightBackMotorController.getMotorTemperature());
    
    //motor inputs
    SmartDashboard.putNumber("drive/lf motor input (%)", leftFrontMotorController.get());
    SmartDashboard.putNumber("drive/lb motor input (%)", leftBackMotorController.get());
    SmartDashboard.putNumber("drive/rf motor input (%)", rightFrontMotorController.get());
    SmartDashboard.putNumber("drive/rb motor input (%)", rightBackMotorController.get());   
}
}

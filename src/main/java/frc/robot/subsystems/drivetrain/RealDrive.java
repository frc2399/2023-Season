package frc.robot.subsystems.drivetrain;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;


public class RealDrive implements DriveIO {
   // Basically copy over the entire rest of the drive subsystem that is "not sim"
   private static CANSparkMax leftFrontMotorController, rightFrontMotorController;
   public static RelativeEncoder leftEncoder, rightEncoder;
   public AHRS ahrs;


   public RealDrive() {
        rightFrontMotorController = new CANSparkMax(DriveConstants.RIGHT_FRONT_DRIVE_CAN_ID, MotorType.kBrushless);
        leftFrontMotorController = new CANSparkMax(DriveConstants.LEFT_FRONT_DRIVE_CAN_ID, MotorType.kBrushless);
        leftEncoder = leftFrontMotorController.getEncoder();
        rightEncoder = rightFrontMotorController.getEncoder();

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


@Override
public void updateForSim() {
    // TODO Auto-generated method stub
    
}
}

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveIO {

    public Rotation2d getGyroAngle();
    public double getRightEncoderMeters();
    public double getLeftEncoderMeters();
    public double getRightEncoderMetersPerSecond();
    public double getLeftEncoderMetersPerSecond();
    public void setMotors(double leftSpeed, double rightSpeed);
    public void setMotorVoltage(double leftVolt, double rightVolt);

    public void updateForSim();
}

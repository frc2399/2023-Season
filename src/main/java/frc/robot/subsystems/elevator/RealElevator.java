package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class RealElevator implements ElevatorIO {

    public static CANSparkMax elevatorMotorControllerRight;
    public static CANSparkMax elevatorMotorControllerLeft;
    public static RelativeEncoder elevatorEncoderRight;
    public static RelativeEncoder elevatorEncoderLeft;
    @Override
    public double getEncoderPosition() {
        return elevatorEncoderRight.getPosition();
    }
    @Override
    public double getEncoderSpeed() {
        return elevatorEncoderRight.getVelocity();
    }
    @Override
    public void setSpeed(double speed) {
        elevatorMotorControllerRight.set(speed);
        
    
    }

}

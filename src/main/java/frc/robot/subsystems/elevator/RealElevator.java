package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.MotorUtil; 

public class RealElevator implements ElevatorIO {

    public static CANSparkMax elevatorMotorControllerRight, elevatorMotorControllerLeft;
    public static RelativeEncoder elevatorEncoderRight;
    public static RelativeEncoder elevatorEncoderLeft;

    public RealElevator()
    {
        elevatorMotorControllerRight = MotorUtil.createSparkMAX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless, 
            Constants.IntakeConstants.NEO_CURRENT_LIMIT, false, false, 0);
        elevatorMotorControllerRight = MotorUtil.createSparkMAX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless, 
            Constants.IntakeConstants.NEO_CURRENT_LIMIT, true, false, 0);

        // initialize motor encoder
        elevatorEncoderRight = elevatorMotorControllerRight.getEncoder();
        elevatorEncoderLeft = elevatorMotorControllerLeft.getEncoder();

        elevatorEncoderRight.setPosition(0); 
        elevatorEncoderLeft.setPosition(0); 

        elevatorMotorControllerLeft.follow(elevatorMotorControllerRight);
    }

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

    @Override
    public void updateForSim() {        
    }

}

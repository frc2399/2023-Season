package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants; 

public class RealElevator implements ElevatorIO {

    public static CANSparkMax elevatorMotorControllerRight;
    public static CANSparkMax elevatorMotorControllerLeft;
    public static RelativeEncoder elevatorEncoderRight;
    public static RelativeEncoder elevatorEncoderLeft;

    public RealElevator()
    {
        elevatorMotorControllerRight = new CANSparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorMotorControllerLeft = new CANSparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);


    // restore factory settings to reset to a known state
    elevatorMotorControllerRight.restoreFactoryDefaults();
    elevatorMotorControllerLeft.restoreFactoryDefaults();

    // set climber motors to coast mode
    elevatorMotorControllerRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorMotorControllerLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // initialize motor encoder
    elevatorEncoderRight = elevatorMotorControllerRight.getEncoder();
    elevatorEncoderLeft = elevatorMotorControllerLeft.getEncoder();

    // invert the motor controllers so climber climbs right
    elevatorMotorControllerRight.setInverted(false);
    elevatorMotorControllerLeft.setInverted(true);

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

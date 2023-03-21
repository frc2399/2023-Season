package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.MotorUtil; 

public class RealElevator implements ElevatorIO {

    public static CANSparkMax elevatorMotorControllerRight, elevatorMotorControllerLeft;
    public static RelativeEncoder elevatorEncoderRight;
    public static RelativeEncoder elevatorEncoderLeft;
    private SparkMaxLimitSwitch topLimitSwitch;
    private SparkMaxLimitSwitch bottomLimitSwitch;

    public RealElevator()
    {
        // lowered slew rate to 0.1 to help with elevator slamming
        elevatorMotorControllerRight = MotorUtil.createSparkMAX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, true, true, 0.1);
        elevatorMotorControllerLeft = MotorUtil.createSparkMAX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, false, true, 0.1);
        
        topLimitSwitch = elevatorMotorControllerLeft.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        bottomLimitSwitch = elevatorMotorControllerLeft.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

        // initialize motor encoder
        elevatorEncoderRight = elevatorMotorControllerRight.getEncoder();
        elevatorEncoderLeft = elevatorMotorControllerLeft.getEncoder();

        elevatorEncoderRight.setPosition(0); 
        elevatorEncoderLeft.setPosition(0); 

        elevatorEncoderRight.setPositionConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION);
        elevatorEncoderLeft.setPositionConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION);

        // dividng by 60 to convert meters per miniute to meters per seconds
        elevatorEncoderRight.setVelocityConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION / 60);
        elevatorEncoderLeft.setVelocityConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION / 60);

        elevatorMotorControllerRight.follow(elevatorMotorControllerLeft);

        //set to false b/c manually do limit switch things so elevator doesn't go crazy, check Elevator.java
        topLimitSwitch.enableLimitSwitch(false);
        bottomLimitSwitch.enableLimitSwitch(true);
    }

    @Override
    public double getEncoderPosition() {
        return elevatorEncoderLeft.getPosition();
    }
    @Override
    public double getEncoderSpeed() {
        return elevatorEncoderLeft.getVelocity();
    }
    
    @Override
    public void setSpeed(double speed) {
        elevatorMotorControllerLeft.set(speed);
    }

    @Override
    public void updateForSim() {
        SmartDashboard.putBoolean("Top Limit Pressed", topLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Bottom Limit Pressed", bottomLimitSwitch.isPressed());    
    }

    @Override
    public void setPosition(double position) {
        elevatorEncoderLeft.setPosition(position);
    }

    @Override
    public double getElevatorCurrent() {
        return elevatorMotorControllerLeft.getOutputCurrent();
    }

    @Override
    public boolean isAtUpperLimit() {
        return topLimitSwitch.isPressed();
    }

    @Override
    public boolean isAtLowerLimit() {
        return bottomLimitSwitch.isPressed();
    }

}

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AnalogEncoder; 

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorController;
    public static RelativeEncoder armEncoder;
    public static AnalogEncoder armAbsoluteEncoder;

    public RealArm() {
        armAbsoluteEncoder = new AnalogEncoder(0);
        armMotorController = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            true, true, 0);
        armEncoder = armMotorController.getEncoder();
        
        armEncoder.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoder.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);

        armEncoder.setPosition(ArmConstants.INITIAL_OFFSET);
    }

    @Override
    public double getAbsoluteEncoderPosition() {
        System.out.println("creating absolute encoder position in real arm file");
        return armAbsoluteEncoder.getAbsolutePosition();
    }

    @Override
    public double getEncoderPosition() {
        return armEncoder.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoder.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorController.set(speed);
    }

    @Override
    public void setPosition(double position) {
        armEncoder.setPosition(position);
    }

    @Override
    public void updateForSim(){
        
    }
    
}

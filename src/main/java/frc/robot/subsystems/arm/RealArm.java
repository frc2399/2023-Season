package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.MotorUtil;
import frc.robot.Constants;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorController;
    public static RelativeEncoder armEncoder;

    public RealArm() {

        armMotorController = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            true, false, 0);
        armEncoder = armMotorController.getEncoder();
        armEncoder.setPosition(0);
        armEncoder.setPositionConversionFactor(Constants.ArmConstants.RADIANS_PER_REVOLUTION);
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
        SmartDashboard.putNumber("ArmSpeed", speed);
    }

    @Override
    public void updateForSim(){
        
    }
    
}

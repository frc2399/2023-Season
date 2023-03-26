package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorController;
    public static RelativeEncoder armEncoder;
    public static DutyCycleEncoder armAbsoluteEncoder;

    public RealArm() {
        // armAbsoluteEncoder = new DutyCycleEncoder(0);
        //Higher slew rate of .75 seconds from 0 to 100% (sparkmax thinks we use this) translates to .2 seconds from 0 to 20% (what we actually use)
        armMotorController = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            true, true, 0.75);
        armEncoder = armMotorController.getEncoder();
        
        armEncoder.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoder.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);

        armEncoder.setPosition(ArmConstants.INITIAL_OFFSET);
    }

    public double getAbsoluteEncoderPosition() {
        return -(armAbsoluteEncoder.getAbsolutePosition() - 0.88) * 2 * Math.PI / 3;
    }

    @Override
    public void periodicUpdate() {
        
    }

    @Override
    public double getEncoderPosition() {
       //return getAbsoluteEncoderPosition();
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
    public double getArmCurrent() {
        return armMotorController.getOutputCurrent();
    }
    
}

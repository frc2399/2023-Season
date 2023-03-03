package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorController;
    public static RelativeEncoder armEncoder;
    public static DutyCycleEncoder armAbsoluteEncoder;

    public RealArm() {
        armAbsoluteEncoder = new DutyCycleEncoder(0);
        armMotorController = MotorUtil.createSparkMAX(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            true, true, 0);
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
        SmartDashboard.putNumber("arm relative encoder position", armEncoder.getPosition());
    }

    @Override
    public double getEncoderPosition() {
        return getAbsoluteEncoderPosition();
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
}

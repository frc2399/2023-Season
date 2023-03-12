package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.MotorUtil;

public class RealIntake implements IntakeIO {

    public static CANSparkMax intakeMotorController;
    public static RelativeEncoder intakeEncoder;
    private double slewRate = 0.6;

    public RealIntake()
    {
        intakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);

        // initialize motor encoder
        intakeEncoder = intakeMotorController.getEncoder();
    }

    @Override
    public void setMotor(double intakeSpeed) {
        intakeMotorController.set(intakeSpeed);
    }

    public double getCurrent()
    {
        return intakeMotorController.getOutputCurrent();
    }

    @Override
    public double getEncoderSpeed() {
        return intakeEncoder.getVelocity();
    }
}

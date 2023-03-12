package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.MotorUtil;

public class RealIntake implements IntakeIO {

    private static CANSparkMax intakeMotorController;
    // private static CANSparkMax rightMotorController;
    private double slewRate = 0.6;

    public RealIntake()
    {
        intakeMotorController = MotorUtil.createSparkMAX(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO550_CURRENT_LIMIT, false, true, slewRate);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        intakeMotorController.set(intakeSpeed);
        // rightMotorController.set(intakeSpeed);
    }

    public double getCurrent()
    {
        return intakeMotorController.getOutputCurrent();
    }
}

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.util.MotorUtil;

public class RealIntake implements IntakeIO {

    private static CANSparkMax leftMotorController;
    // private static CANSparkMax rightMotorController;
    private double slewRate = 0.6;

    public RealIntake()
    {
        leftMotorController = MotorUtil.createSparkMAX(IntakeConstants.LEFT_INTAKE_MOTOR_ID, MotorType.kBrushless, 
            IntakeConstants.NEO550_CURRENT_LIMIT, false, true, slewRate);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        leftMotorController.set(intakeSpeed);
        // rightMotorController.set(intakeSpeed);
    }
}

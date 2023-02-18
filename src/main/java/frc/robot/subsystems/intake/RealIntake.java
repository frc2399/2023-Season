package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class RealIntake implements IntakeIO {

    private static CANSparkMax leftMotorController;
    // private static CANSparkMax rightMotorController;
    private double slewRate = 0.6;

    public RealIntake()
    {
        leftMotorController = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR_ID, MotorType.kBrushless);
        // rightMotorController = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, MotorType.kBrushless);
        leftMotorController.setInverted(false);
        // rightMotorController.setInverted(true);
        leftMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // rightMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // built in slew rate for spark max
        leftMotorController.setOpenLoopRampRate(slewRate);
        // rightMotorController.setOpenLoopRampRate(slewRate);
        leftMotorController.setSmartCurrentLimit(IntakeConstants.NEO_CURRENT_LIMIT);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        leftMotorController.set(intakeSpeed);
        // rightMotorController.set(intakeSpeed);
    }
}

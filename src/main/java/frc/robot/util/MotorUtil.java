// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MotorUtil {

    public static CANSparkMax createSparkMAX(int id, MotorType motortype, int stallLimit, boolean isInverted, boolean isIdleBreak, double slewRate) {
        CANSparkMax sparkMAX = createSparkMAX(id, motortype, stallLimit, isIdleBreak, slewRate);
        sparkMAX.setInverted(isInverted);
        return sparkMAX;
    }

    public static CANSparkMax createSparkMAX(int id, MotorType motortype, int stallLimit, boolean isIdleBreak, double slewRate) {
        CANSparkMax sparkMAX = new CANSparkMax(id, motortype);
        sparkMAX.restoreFactoryDefaults();
        // sparkMAX.enableVoltageCompensation(voltageCompensation);
        sparkMAX.setSmartCurrentLimit(stallLimit);

        if(isIdleBreak) {
            sparkMAX.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }
        else {
            sparkMAX.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }

        // built in slew rate for spark max
        sparkMAX.setOpenLoopRampRate(slewRate);

        sparkMAX.burnFlash();
        return sparkMAX;
    }
}

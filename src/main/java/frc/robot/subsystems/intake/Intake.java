// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    
    intakeIO = io;
        
  }

  public void setMotor(double intakeSpeed) {
  
    intakeIO.setMotor(intakeSpeed);
    SmartDashboard.putNumber("intake speed", intakeSpeed);
}

  public double getCurrent() {
    return intakeIO.getCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake current", getCurrent());
    
  }
}

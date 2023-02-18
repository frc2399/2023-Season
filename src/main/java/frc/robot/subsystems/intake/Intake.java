// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    
    intakeIO = io;
        
  }

  public void setMotor(double intakeSpeed) {
  
    intakeIO.setMotor(intakeSpeed);
}

  //Intake methods (different combos of spinny spin and pneumatics)
  public void intakeBothArms() {

    setMotor(Constants.IntakeConstants.INTAKE_IN_SPEED);

  }

  public void intakeRight() {

    setMotor(Constants.IntakeConstants.INTAKE_IN_SPEED);

  }

  public void intakeLeft() {

    setMotor(Constants.IntakeConstants.INTAKE_IN_SPEED);

  }

  public void outtake() {

    setMotor(Constants.IntakeConstants.INTAKE_OUT_SPEED);
    
  }

  public void drop() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}

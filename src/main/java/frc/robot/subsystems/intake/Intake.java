// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO;
  //private DoubleSolenoid leftIntakeSolenoid;
  private DoubleSolenoid rightIntakeSolenoid;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    intakeIO = io;
    rightIntakeSolenoid = new DoubleSolenoid(IntakeConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, 
    IntakeConstants.FORWARD_CHANNEL_SOLENOID_ID, IntakeConstants.REVERSE_CHANNEL_SOLENOID_ID);
        
  }

  //Pneumatic methods
  // public void closeLeft() {

  //   if (RobotBase.isSimulation()) {
  //     leftSolenoidSim.setOutput(true);
  //   }
  //   else {
  //     leftIntakeSolenoid.set(Value.kForward);
  //   }  

  // }

  public void closeRight() {

      rightIntakeSolenoid.set(Value.kForward);

  }

  // public void openLeft() {

  //   if (RobotBase.isSimulation()) {
  //     leftSolenoidSim.setOutput(false);
  //   }
  //   else {
  //     leftIntakeSolenoid.set(Value.kOff);
  //   }

  // }

  public void openRight() {

    rightIntakeSolenoid.set(Value.kReverse);

  }

  public void setMotor(double intakeSpeed) {
  
    intakeIO.setMotor(intakeSpeed);
}

  //Intake methods (different combos of spinny spin and pneumatics)
  public void intakeBothArms() {

    //closeLeft();
    closeRight();
    setMotor(Constants.IntakeConstants.INTAKE_IN_SPEED);

  }

  public void intakeRight() {

    // openLeft();
    closeRight();
    setMotor(Constants.IntakeConstants.INTAKE_IN_SPEED);

  }

  public void intakeLeft() {

    openRight();
    // closeLeft();
    setMotor(Constants.IntakeConstants.INTAKE_IN_SPEED);

  }

  public void outtake() {
    // closeLeft();
    closeRight();
    setMotor(Constants.IntakeConstants.INTAKE_OUT_SPEED);
  }

  public void drop() {

    // openLeft();
    openRight();

  }

  // public boolean isLeftOpen() {
  //   if (RobotBase.isSimulation()) {
  //     return leftSolenoidSim.getOutput();
  //   }
  //   else {
  //     return (leftIntakeSolenoid.get()== Value.kForward);

  //   }
  // }

  public boolean isRightOpen() {
      return (rightIntakeSolenoid.get()== Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Left Intake Open? ", isLeftOpen());
    SmartDashboard.putBoolean("Right Intake Open? ", isRightOpen());
  }
}

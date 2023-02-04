// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  SlewRateLimiter filter;
  private static CANSparkMax leftMotorController;
  private static CANSparkMax rightMotorController;
  //private DoubleSolenoid leftIntakeSolenoid;
  private DoubleSolenoid rightIntakeSolenoid;


  /** Creates a new Intake. */
  public Intake() {

    leftMotorController = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR_ID, MotorType.kBrushless);
    rightMotorController = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, MotorType.kBrushless);
    rightIntakeSolenoid = new DoubleSolenoid(IntakeConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, 
    IntakeConstants.FORWARD_CHANNEL_SOLENOID_ID, IntakeConstants.REVERSE_CHANNEL_SOLENOID_ID);
    leftMotorController.setInverted(false);
    rightMotorController.setInverted(true);
    leftMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);

   filter = new SlewRateLimiter(IntakeConstants.INTAKE_SLEW_RATE);
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

  //Intake spinny spin methods


  public void setSpeed(double speed) {
    double slewSpeed = filter.calculate(speed);
    SmartDashboard.putNumber ("intakeSpeed",slewSpeed);

      leftMotorController.set(slewSpeed);
      rightMotorController.set(slewSpeed);

  }

  //Intake methods (different combos of spinny spin and pneumatics)
  public void intakeBothArms() {

    //closeLeft();
    closeRight();
    setSpeed(Constants.IntakeConstants.INTAKE_SPEED);

  }

  public void intakeRight() {

    // openLeft();
    closeRight();
    setSpeed(Constants.IntakeConstants.INTAKE_SPEED);

  }

  public void intakeLeft() {

    openRight();
    // closeLeft();
    setSpeed(Constants.IntakeConstants.INTAKE_SPEED);

  }

  public void outtake() {
    // closeLeft();
    closeRight();
    setSpeed(Constants.IntakeConstants.OUTTAKE_SPEED);
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

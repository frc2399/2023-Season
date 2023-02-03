// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static double intakeSpeed;
  SlewRateLimiter filter;
  private static CANSparkMax leftMotorController;
  private static CANSparkMax rightMotorController;
  private DoubleSolenoid leftIntakeSolenoid;
  private DoubleSolenoid rightIntakeSolenoid;

  private REVPHSim simPH;
  private SolenoidSim leftSolenoidSim;
  private SolenoidSim rightSolenoidSim;
  private DCMotorSim leftMotorSim;
  private DCMotorSim rightMotorSim;

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

    if (RobotBase.isSimulation()) {
      simPH = new REVPHSim();
      leftSolenoidSim = new SolenoidSim(simPH, 1);
      rightSolenoidSim = new SolenoidSim(simPH, 2);
      
    }
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

    if (RobotBase.isSimulation()) {
      rightSolenoidSim.setOutput(true);
    }
    else {
      rightIntakeSolenoid.set(Value.kForward);
    }

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

    if (RobotBase.isSimulation()) {
      rightSolenoidSim.setOutput(false);
    }
    else {
    rightIntakeSolenoid.set(Value.kReverse);
    }

  }

  //Intake spinny spin methods
  public void spinIn(double speed) {
    speed = filter.calculate(1.0);
    SmartDashboard.putNumber ("intakeSpeed",speed);

    if (RobotBase.isSimulation()) {
      leftMotorSim.setInput (speed);
      rightMotorSim.setInput(speed);
    }
    else {
      leftMotorController.set(speed);
      rightMotorController.set(speed);
    }

  }

  public void setSpeed(double speed) {
    double slew_speed = filter.calculate(speed);
    //double slew_speed = speed;
    SmartDashboard.putNumber ("intakeSpeed",slew_speed);

    if (RobotBase.isSimulation()) {
      leftMotorSim.setInput (slew_speed);
      rightMotorSim.setInput(slew_speed);
    }
    else {
      leftMotorController.set(slew_speed);
      rightMotorController.set(slew_speed);
    }

  }

  public void spitOut(double speed) {
    speed = filter.calculate(1.0);
    if (RobotBase.isSimulation()) {
      leftMotorSim.setInput(-speed);
      rightMotorSim.setInput(-speed);
    }
    else {
      leftMotorController.set(-speed);
      rightMotorController.set(-speed);
    }

  }

  //Intake methods (different combos of spinny spin and pneumatics)
  public void intakeBothArms() {

    //closeLeft();
    closeRight();
    spinIn(intakeSpeed);

  }

  public void intakeRight() {

    // openLeft();
    closeRight();
    spinIn(intakeSpeed);

  }

  public void intakeLeft() {

    openRight();
    // closeLeft();
    spinIn(intakeSpeed);

  }

  public void outtake() {
    // closeLeft();
    closeRight();
    spitOut(intakeSpeed);
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
    if (RobotBase.isSimulation()) {
      return rightSolenoidSim.getOutput();
    }
    else {
      return (rightIntakeSolenoid.get()== Value.kForward);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Left Intake Open? ", isLeftOpen());
    SmartDashboard.putBoolean("Right Intake Open? ", isRightOpen());
  }
}

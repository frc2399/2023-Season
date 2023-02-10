// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private static double intakeSpeed;
  private static CANSparkMax leftMotorController;
  private static CANSparkMax rightMotorController;
  //private DoubleSolenoid leftIntakeSolenoid;
  private DoubleSolenoid rightIntakeSolenoid;
  private SolenoidSim leftSolenoidSim;
  private SolenoidSim rightSolenoidSim;
  private DCMotorSim leftMotorSim;
  private DCMotorSim rightMotorSim;
  private double slewRate = 0.6;


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
    // built in slew rate for spark max
    leftMotorController.setOpenLoopRampRate(slewRate);
    rightMotorController.setOpenLoopRampRate(slewRate);
    

    if (RobotBase.isSimulation()) {
      REVPHSim simPH = new REVPHSim();
      leftSolenoidSim = new SolenoidSim(simPH, 1);
      rightSolenoidSim = new SolenoidSim(simPH, 2);
      leftMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
      rightMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
      
    }
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
  
    if (RobotBase.isSimulation()) {
      leftMotorSim.setInput(intakeSpeed);
      rightMotorSim.setInput(intakeSpeed);
    }
    else {
      leftMotorController.set(intakeSpeed);
      rightMotorController.set(intakeSpeed);
    }
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

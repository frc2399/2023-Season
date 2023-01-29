// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static CANSparkMax leftMotorController;
  private static CANSparkMax rightMotorController;
  private Solenoid leftIntakeSolenoid;
  private Solenoid rightIntakeSolenoid;

  private REVPHSim simPH;
  private SolenoidSim leftSolenoidSim;
  private SolenoidSim rightSolenoidSim;
  private DCMotorSim leftMotorSim;
  private DCMotorSim rightMotorSim;

  /** Creates a new Intake. */
  public Intake() {

    leftMotorController = new CANSparkMax(IntakeConstants.LEFT_INTAKE_MOTOR_ID, MotorType.kBrushless);
    rightMotorController = new CANSparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, MotorType.kBrushless);
    leftIntakeSolenoid = new Solenoid(IntakeConstants.PH_ADDRESS, PneumaticsModuleType.REVPH, IntakeConstants.EXTEND_INTAKE_ARM_LEFT);
    rightIntakeSolenoid = new Solenoid(IntakeConstants.PH_ADDRESS, PneumaticsModuleType.REVPH, IntakeConstants.EXTEND_INTAKE_ARM_RIGHT);

    if (RobotBase.isSimulation()) {
      simPH = new REVPHSim();
      leftSolenoidSim = new SolenoidSim(simPH, 1);
      rightSolenoidSim = new SolenoidSim(simPH, 2);
      leftMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
      rightMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
      
    }

  }

  //Pneumatic methods
  public void closeLeft() {

    if (RobotBase.isSimulation()) {
      leftSolenoidSim.setOutput(true);
    }
    else {
      leftIntakeSolenoid.set(true);
    }  

  }

  public void closeRight() {

    if (RobotBase.isSimulation()) {
      rightSolenoidSim.setOutput(true);
    }
    else {
      rightIntakeSolenoid.set(true);
    }

  }

  public void openLeft() {

    if (RobotBase.isSimulation()) {
      leftSolenoidSim.setOutput(false);
    }
    else {
      leftIntakeSolenoid.set(false);
    }

  }

  public void openRight() {

    if (RobotBase.isSimulation()) {
      rightSolenoidSim.setOutput(false);
    }
    else {
    rightIntakeSolenoid.set(false);
    }

  }

  //Intake spinny spin methods
  public void spinnySpinIn() {

    if (RobotBase.isSimulation()) {
      leftMotorSim.setInput(1.0);
      rightMotorSim.setInput(1.0);
    }
    else {
      leftMotorController.set(1);
      rightMotorController.set(1);
    }

  }

  public void spinnySpinOut() {

    if (RobotBase.isSimulation()) {
      leftMotorSim.setInput(-1.0);
      rightMotorSim.setInput(-1.0);
    }
    else {
      leftMotorController.set(-1);
      rightMotorController.set(-1);
    }

  }

  //Intake methods (different combos of spinny spin and pneumatics)
  public void intakeBothArms() {

    closeLeft();
    closeRight();
    spinnySpinIn();

  }

  public void intakeRight() {

    openLeft();
    closeRight();
    spinnySpinIn();

  }

  public void intakeLeft() {

    openRight();
    closeLeft();
    spinnySpinIn();

  }

  public void outtake() {
    closeLeft();
    closeRight();
    spinnySpinOut();
  }

  public void drop() {

    openLeft();
    openRight();

  }

  public boolean isLeftOpen() {
    if (RobotBase.isSimulation()) {
      return leftSolenoidSim.getOutput();
    }
    else {
      return leftIntakeSolenoid.get();
    }
  }

  public boolean isRightOpen() {
    if (RobotBase.isSimulation()) {
      return rightSolenoidSim.getOutput();
    }
    else {
      return rightIntakeSolenoid.get();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Left Intake Open? ", isLeftOpen());
    SmartDashboard.putBoolean("Right Intake Open? ", isRightOpen());

  }
}

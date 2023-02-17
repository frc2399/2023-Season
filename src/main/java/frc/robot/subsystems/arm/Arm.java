// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private ArmIO armIO;
  private double targetAngle = - Math.PI/2; 
  
  public Arm(ArmIO io) {
    armIO = io;
  }

  @Override
  public void periodic() {
    armIO.updateForSim();  
  }
  
  public double getEncoderPosition() {
    return armIO.getEncoderPosition();
  }

  public double getEncoderSpeed() {
    return armIO.getEncoderSpeed();
  }

  public void setSpeed(double speed) {
    armIO.setSpeed(speed);
  }

  public double getTargetAngle() {
    return targetAngle; 
  }

  public void setTargetAngle(double angle) {
    System.out.println("Set target to " + angle);
    targetAngle = angle; 
  }
}

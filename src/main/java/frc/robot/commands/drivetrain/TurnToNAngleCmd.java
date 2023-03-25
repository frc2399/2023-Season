// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.util.PIDUtil;

/**
 * Turns the robot to a target angle (value is passed through the constructor)
 * factors in the robot's current angle to calculate the angle the robot needs to turn in order to reach its target
 * Used in turnAuto sequential auton command
 */

public class TurnToNAngleCmd extends CommandBase {
  /** Creates a new TurnToNAngle. */
  public double targetAngle;
  private final DriveTrain m_driveTrain;
  private double currentAngle;
  private double range = Units.degreesToRadians(5);
  private double kP = .2;

  public TurnToNAngleCmd(double targetAngle, DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("TurnToNAngle initialized, targetAngle: " + targetAngle);
    // SmartDashboard.putNumber("target angle", targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    currentAngle = m_driveTrain.getGyroAngle().getRadians();
    currentAngle = modAngle(currentAngle);

    double error = targetAngle - currentAngle;
    error = modAngle(error);
    // SmartDashboard.putNumber("error", error);

    double outputSpeed = kP * error;
    outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);

    m_driveTrain.setMotors(-outputSpeed, outputSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.setMotors(0, 0);
    DataLogManager.log("TurnToNangle ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PIDUtil.checkWithinRange(targetAngle, currentAngle, range);
  }

  public double modAngle(double value) {
    return ((value + Math.PI) % (Math.PI * 2)) - Math.PI;
  }
}

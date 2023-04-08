// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
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
  private double errorTolerance = Units.degreesToRadians(1);
  private double kP = .15;
  private SlewRateLimiter turnLimiter;
  double error; 
  private double transformedTargetAngle;

  public TurnToNAngleCmd(double targetAngle, DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.turnLimiter = new SlewRateLimiter(1.5);
    DataLogManager.log("TurnToNAngle initialized, targetAngle: " + targetAngle);
    System.out.println(DriverStation.getAlliance().toString());

    if (Robot.allianceColor == DriverStation.Alliance.Red) {
      transformedTargetAngle = -targetAngle;
    }
    else
    {
      transformedTargetAngle = targetAngle;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = m_driveTrain.getPoseMeters().getRotation().getRadians();

    error = modAngle(transformedTargetAngle - currentAngle);
    SmartDashboard.putNumber("turn to angle error", error);

    double outputSpeed = kP * error;
    outputSpeed = MathUtil.clamp(outputSpeed, -0.5, 0.5);
    outputSpeed = turnLimiter.calculate(outputSpeed);
    //created a min speed to overcome small errors
    outputSpeed += 0.085 * Math.signum(outputSpeed);

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
    return PIDUtil.checkWithinRange(0, error, errorTolerance);
  }

  public double modAngle(double value) {
    value = (value + Math.PI) % (Math.PI * 2);  // Take "remainder" (https://stackoverflow.com/a/2172061)
    value = value < 0 ? value + Math.PI * 2 : value;  // If less then 0, add the value to make it "modulus"
    return value - Math.PI;  // Subtract PI to make the angle in the range -PI to PI
  }
}

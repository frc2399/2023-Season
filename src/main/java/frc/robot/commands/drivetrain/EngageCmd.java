// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.DriveTrain;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class EngageCmd extends CommandBase {

  private DriveTrain drivetrain;

  private double currentAngle;
  private double drivePower;
  private double derivative;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public EngageCmd(DriveTrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivePower = 1/180. * drivetrain.getGyroPitch();
    drivePower = Math.max(Math.min(drivePower, 0.2), -0.2);
  
    drivetrain.setMotors(drivePower, drivePower);

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setMotors(0, 0);
  }

  @Override
  public boolean isFinished() {
   // return Math.abs(error) < Constants.DriveConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    return drivetrain.getGyroPitchRate() < -15;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class EngageCmd extends CommandBase {

  private DriveTrain drivetrain;

  private double drivePower;
  private double speedLimit;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public EngageCmd(DriveTrain drivetrain, double speedLimit) {
    this.drivetrain = drivetrain;
    this.speedLimit = speedLimit;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivePower = 0.01 * drivetrain.getGyroPitch();
    // drivePower = Math.max(Math.min(drivePower, 0.3), -0.3);
    drivePower = Math.max(Math.min(drivePower, speedLimit), -speedLimit);
    drivePower = -drivePower;

    if (Math.abs(drivetrain.getGyroPitchRate()) > 15) {
      drivePower = 0;
    }
  
   //converting drivePower into voltage 
    drivePower *= 12; 
    drivetrain.setMotorVoltage(drivePower, drivePower);

  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setMotorVoltage(0, 0);
  }

  @Override
  public boolean isFinished() {
   // return Math.abs(error) < Constants.DriveConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    return false;
  }
}
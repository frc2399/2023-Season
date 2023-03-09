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

  private double error;
  private double currentAngle;
  private double drivePower;
  private double derivative;
  private LinearFilter derivativeCalculator = LinearFilter.backwardFiniteDifference(1, 5, 0.02);

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public EngageCmd(DriveTrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = drivetrain.getGyroPitch();
    derivative = derivativeCalculator.calculate(currentAngle);

    error = Constants.DriveConstants.BEAM_BALANCED_GOAL_DEGREES - currentAngle;
    drivePower = -Math.min(1/180. * error, 1);

    // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    if (drivePower < 0) {
      drivePower *= Constants.DriveConstants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }

    // Limit the max power
    if (Math.abs(drivePower) > 0.2) {
      drivePower = Math.copySign(0.2, drivePower);
    }


    drivetrain.setMotors(-drivePower, -drivePower);

    SmartDashboard.putNumber("derivative", derivative);
    
    // // Debugging Print Statments
    // System.out.println("Current Angle: " + currentAngle);
    // System.out.println("Error " + error);
    // System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // return Math.abs(error) < Constants.DriveConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    return derivative < -0.05;
  }
}
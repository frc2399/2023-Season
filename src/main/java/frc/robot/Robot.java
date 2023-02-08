// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {

  
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    robotContainer = new RobotContainer();

    PPRamseteCommand.setLoggingCallbacks(
      (PathPlannerTrajectory activeTrajectory) -> {
          // Log current trajectory
      },
      (Pose2d targetPose) -> {
          // Log target pose
      },
      (ChassisSpeeds setpointSpeeds) -> {
          // Log setpoint ChassisSpeeds
          // System.out.println("log setpoint ChassisSpeeds " + setpointSpeeds);
          DifferentialDriveWheelSpeeds targetWheelSpeeds =
          DriveConstants.kDriveKinematics.toWheelSpeeds(setpointSpeeds);
          // ChassisSpeeds chassisSpeeds = 
          // DriveConstants.kDriveKinematics.toChassisSpeeds(targetWheelSpeeds);
          double leftTargetVelocity = targetWheelSpeeds.leftMetersPerSecond;
          double rightTargetVelocity = targetWheelSpeeds.rightMetersPerSecond;

          SmartDashboard.putNumber("Left Target Velocity", leftTargetVelocity);
          SmartDashboard.putNumber("Right Target Velocity", rightTargetVelocity);

      },
      (Translation2d translationError, Rotation2d rotationError) -> {
          // Log path following error
          //TODO what is this?
      }
);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    // double speed = Elevator.elevatorJoystick.getRawAxis(0);
    // Elevator.motorController.set(speed); 

  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("cone mode", RobotContainer.coneMode);
  }

  @Override

  public void autonomousInit(){
        m_autonomousCommand = robotContainer.getAutonomousCommand();
        System.out.println("We are in auton init!" + m_autonomousCommand);

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
  }

  @Override

  public void disabledPeriodic() {
    //Elevator.motorController.set(0.0);
    // robotContainer.elevator.setSpeed(0);
  }
}

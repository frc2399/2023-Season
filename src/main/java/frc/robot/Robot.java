// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.CurvatureDriveCmd;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.drivetrain.DriveTrain;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command m_autonomousCommand;
  public static Alliance allianceColor;
  private SparkMaxPIDController controller;

  @Override
  public void robotInit() {

    robotContainer = new RobotContainer();
    // if (!RobotBase.isSimulation()) {
    //   DataLogManager.start();
    //   DriverStation.startDataLog(DataLogManager.getLog());
    // }

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    PPRamseteCommand.setLoggingCallbacks(
      (PathPlannerTrajectory activeTrajectory) -> {
          // Log current trajectory
          DriveTrain.field.getObject("traj").setTrajectory(activeTrajectory);
      },
      (Pose2d targetPose) -> {
          // Log target pose
      },
      (ChassisSpeeds setpointSpeeds) -> {
        
          DifferentialDriveWheelSpeeds targetWheelSpeeds =
          DriveConstants.kDriveKinematics.toWheelSpeeds(setpointSpeeds);

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
    // SmartDashboard.putString("branch and date", MyVersion.GIT_BRANCH + " " + MyVersion.GIT_DATE);
    Shuffleboard.getTab("Driver").add("robot/branch info", MyVersion.GIT_BRANCH + " " + MyVersion.GIT_DATE + " " + MyVersion.GIT_SHA);
    SmartDashboard.putData("PDP", RobotContainer.pdp);

    CommandScheduler.getInstance( ).onCommandInitialize(cmd -> DataLogManager.log(cmd.getName( ) + ": Init"));
    CommandScheduler.getInstance( ).onCommandInterrupt(cmd -> DataLogManager.log(cmd.getName( ) + ": Interrupted"));
    CommandScheduler.getInstance( ).onCommandFinish(cmd -> DataLogManager.log(cmd.getName( ) + ": End"));
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
    SmartDashboard.putBoolean("robot/cone cube mode", RobotContainer.coneMode);
    SmartDashboard.putString("robot/node height", RobotContainer.angleHeight.toString());
  }

  @Override
  public void testInit() {
    SmartDashboard.putNumber("reference arm angle (degrees)", SmartDashboard.getNumber("reference arm angle (degrees)", 0));
    SmartDashboard.putNumber("arm P gain", SmartDashboard.getNumber("arm P gain", 0));
    // SmartDashboard.putNumber("arm D gain", 0);
    // SmartDashboard.putNumber("arm FF gain", 0);
    SmartDashboard.putNumber("gravity compensation", SmartDashboard.getNumber("gravity compensation", 0));


    controller = RealArm.armMotorController.getPIDController();
    controller.setP(SmartDashboard.getNumber("arm P gain", 0));
    controller.setI(0);
    controller.setD(SmartDashboard.getNumber("arm D gain", 0));
    controller.setFF(0); 
    controller.setFeedbackDevice(RealArm.armEncoder);
   

  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("arm output", RealArm.armMotorController.get());
    controller.setReference(
      Units.degreesToRadians(SmartDashboard.getNumber("reference arm angle (degrees)", 0)), 
      CANSparkMax.ControlType.kPosition, 
      0, 
      SmartDashboard.getNumber("gravity compensation", 0) *
         Math.cos(robotContainer.arm.getEncoderPosition()),
      SparkMaxPIDController.ArbFFUnits.kPercentOut);
   
  }

  @Override

  public void autonomousInit(){
        m_autonomousCommand = robotContainer.getAutonomousCommand();
        DataLogManager.log("We are in auton init!" + m_autonomousCommand);

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        allianceColor = DriverStation.getAlliance();
        SmartDashboard.putString("robot/alliance color", DriverStation.getAlliance().toString()); 
  }

  @Override
  public void teleopInit() {
    // after auton robot will not stay in slow mode
    CurvatureDriveCmd.isSlow = false;
  }

  @Override

  public void disabledPeriodic() {
    //Elevator.motorController.set(0.0);
    // robotContainer.elevator.setSpeed(0);
  }

  @Override
  public void disabledInit()
  {
    //Disables arm and elevator PID loops so it won't remember/try to get to the last setpoint
    //Otherwise, if the arm fell after disabling, it would go up really quickly on enabling
    //Also disables gravity compensation b/c no command with gravity compensation running after disable
    RobotContainer.arm.disable();
    RobotContainer.elevator.disable();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.TurnToNAngleCmd;
import frc.robot.commands.intake.IntakeForGivenTime;
import frc.robot.commands.robot.PlaceConeOnNode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.PathUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuton extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuton. */

   //mirror if on red alliance
   boolean useAllianceColor = true;

   PathPlannerTrajectory secondCone = PathPlanner.loadPath("Second-Cone-Auton", 
   new PathConstraints(1, 1), true);
        
  
  public TwoPieceAuton(DriveTrain driveTrain, Elevator elevator, Intake intake, Arm arm) {
    Command coneTwo = 
    new PPRamseteCommand(
      secondCone,
      () -> driveTrain.getPoseMeters(), // Pose supplier
      new RamseteController(),
      new SimpleMotorFeedforward(
        Constants.DriveConstants.ks,
        Constants.DriveConstants.kv,
        Constants.DriveConstants.ka
      ),
      Constants.DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
      () -> driveTrain.getWheelSpeedsMetersPerSecond(), // DifferentialDriveWheelSpeeds supplier
      new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0
        // will only use feedforwards.
      new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
      (left, right) -> driveTrain.setMotorVoltage(left, right), // voltage
      useAllianceColor, //uses alliance color to determine starting position
      driveTrain // Requires this drive subsystem
    );

    addCommands(
      RobotContainer.resetArmAndElevatorEncoderCommand(arm, elevator),
      new DriveForwardGivenDistance(-0.20, driveTrain),
      new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE),
      new PrintCommand("place cone on node finished"),
      // leaves community
      new DriveForwardGivenDistance(-4.2, driveTrain),
      new TurnToNAngleCmd(Units.degreesToRadians(0), driveTrain),
      // lower arm
      RobotContainer.makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_UP_INTAKE_ANGLE, ElevatorConstants.CONE_UP_INTAKE_HEIGHT),
      // drives and intakes cone off ground
      new ParallelCommandGroup(
          new DriveForwardGivenDistance(0.3, driveTrain),
          new IntakeForGivenTime(intake, IntakeConstants.CONE_IN_SPEED, 0.7)
      ),
      RobotContainer.makeSetPositionArmAndElevatorCommand(ArmConstants.TURTLE_ANGLE, 0),
      coneTwo,
      new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE)
      
    );
  }
}

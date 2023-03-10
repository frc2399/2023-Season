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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
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
        
  public TwoPieceAuton(DriveTrain driveTrain, Elevator elevator, Intake intake, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    PathPlannerTrajectory twoPiecePath = PathPlanner.loadPath("Two-Cone Auton", 
      new PathConstraints(1, 1), true);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("lower arm", RobotContainer.makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_UP_INTAKE_ANGLE, ElevatorConstants.MIN_ELEVATOR_HEIGHT));
    eventMap.put("Intake", new IntakeForGivenTime(intake, IntakeConstants.CONE_IN_SPEED, 2));
        
    Command eventTesting = 
      new PPRamseteCommand(
        twoPiecePath,
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

        FollowPathWithEvents twoPieceAuton = new FollowPathWithEvents(
            eventTesting,
            twoPiecePath.getMarkers(),
            eventMap
        );

    addCommands(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
          //driveTrain.resetOdometry(twoPiecePath.getInitialPose());
          driveTrain.resetOdometry(PathUtil.getInitialPoseForAlliance(twoPiecePath));
            }, driveTrain),
        // scores cone to top node
      new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE),
      // runs two piece auton path that picks up second cone
      twoPieceAuton,
      // scores second cone on top node
      new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE)
      );

  }
}

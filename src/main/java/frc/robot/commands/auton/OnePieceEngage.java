package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.commands.intake.IntakeForGivenTime;
import frc.robot.commands.robot.PlaceConeOnNode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class OnePieceEngage extends SequentialCommandGroup {

    public OnePieceEngage(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm) {

        addCommands(
            // RobotContainer.makeSetPositionCommand(elevator, ElevatorConstants.CONE_TOP_NODE_HEIGHT),
            // RobotContainer.makeSetPositionCommand(arm, 0),
            // new PrintCommand("arm angle: " + arm.getGoal()),
            // new IntakeForGivenTime(intake, IntakeConstants.INTAKE_OUT_SPEED, 1),
            new PlaceConeOnNode(intake, elevator, arm, 0),
            RobotContainer.makeSetPositionCommand(arm, ArmConstants.MAX_ARM_ANGLE),
            RobotContainer.makeSetPositionCommand(elevator, ElevatorConstants.MIN_ELEVATOR_HEIGHT),
            new DriveForwardGivenDistance(0.3, 4, driveTrain),
            new DriveForwardGivenDistance(-0.3, 2, driveTrain),
            new EngageCmd()
          );
    }

}

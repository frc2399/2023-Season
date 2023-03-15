package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.intake.IntakeForGivenTime;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class PlaceConeOnNode extends SequentialCommandGroup {

    public PlaceConeOnNode(Intake intake, Elevator elevator, Arm arm, double height, double armAngle) {

        addCommands(
            RobotContainer.makeSetPositionCommand(arm, ArmConstants.TURTLE_ANGLE),
            new WaitUntilCommand(() -> arm.atGoal()),
            RobotContainer.makeSetPositionCommand(elevator, height),
            new WaitUntilCommand(() -> elevator.atGoal()),
            new PrintCommand("elevator at goal"),
            RobotContainer.makeSetPositionCommand(arm, armAngle),
            new WaitUntilCommand(() -> arm.atGoal()),
            new PrintCommand("arm at goal"),
            new DriveForwardGivenDistance(0.08, RobotContainer.driveTrain),
            new IntakeForGivenTime(intake, IntakeConstants.CONE_OUT_SPEED, 1),
            RobotContainer.makeSetPositionCommand(arm, ArmConstants.TURTLE_ANGLE),
            RobotContainer.makeSetPositionCommand(elevator, 0),
            new WaitUntilCommand(() -> arm.atGoal()),
            new WaitUntilCommand(() -> elevator.atGoal())
          );
    }

}


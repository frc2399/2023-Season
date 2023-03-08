package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.IntakeForGivenTime;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class PlaceConeOnNode extends SequentialCommandGroup {

    public PlaceConeOnNode(Intake intake, Elevator elevator, Arm arm, double height, double armAngle) {

        addCommands(
            RobotContainer.makeSetPositionCommand(elevator, height),
            RobotContainer.makeSetPositionCommand(arm, armAngle),
            new WaitUntilCommand(() -> arm.atGoal()),
            new WaitUntilCommand(() -> elevator.atGoal()),
            new PrintCommand("arm angle: " + arm.getGoal()),
            new IntakeForGivenTime(intake, IntakeConstants.CONE_OUT_SPEED, 1),
            RobotContainer.makeSetPositionCommand(arm, ArmConstants.MAX_ARM_ANGLE),
            RobotContainer.makeSetPositionCommand(elevator, ElevatorConstants.MIN_ELEVATOR_HEIGHT),
            new WaitUntilCommand(() -> arm.atGoal()),
            new WaitUntilCommand(() -> elevator.atGoal())
          );
    }

}


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

public class IntakeConeFromGround extends SequentialCommandGroup {

    public IntakeConeFromGround(Intake intake, Elevator elevator, Arm arm) {

        addCommands(
            RobotContainer.makeSetPositionCommand(arm, 0),
            new WaitUntilCommand(() -> arm.atGoal()),
            new PrintCommand("arm angle: " + arm.getGoal()),
            new IntakeForGivenTime(intake, IntakeConstants.CONE_IN_SPEED, 1),
            RobotContainer.makeSetPositionCommand(arm, ArmConstants.MAX_ARM_ANGLE),
            new WaitUntilCommand(() -> arm.atGoal())
          );
    }

}


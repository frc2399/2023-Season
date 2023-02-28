package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
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
            new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT),
            new DriveForwardGivenDistance(0.3, 4, driveTrain),
            new DriveForwardGivenDistance(-0.3, 2, driveTrain),
            new EngageCmd()
          );
    }

}

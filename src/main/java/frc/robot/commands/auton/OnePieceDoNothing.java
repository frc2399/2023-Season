package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.robot.PlaceConeOnNode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class OnePieceDoNothing extends SequentialCommandGroup {

    public OnePieceDoNothing(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm) {

        addCommands(
            RobotContainer.resetArmAndElevatorEncoderCommand(arm, elevator),
            new DriveForwardGivenDistance(-0.20, driveTrain),
            new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE)
          );
    }

}

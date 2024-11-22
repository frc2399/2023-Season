package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
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
            // TODO: test
            RobotContainer.resetArmAndElevatorEncoderCommand(arm, elevator),
            new DriveForwardGivenDistance(-0.2, driveTrain),
            new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE),
            new PrintCommand("place cone on node finished"),
            // wait one second before driving back on charging station so charging station becomes level again
            new WaitCommand(1),
            // drive on charging station
            new DriveForwardGivenDistance(-2, driveTrain),
            new PrintCommand("drive forward given distance finished "),

            // balances robot when it's on charging station
            new EngageCmd(driveTrain, 0.2)
          );
    }

}

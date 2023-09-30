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
import frc.robot.commands.robot.PlaceCubeOnNode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class OneCubeCommunityEngage extends SequentialCommandGroup {

    public OneCubeCommunityEngage(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm) {

        addCommands(
            RobotContainer.resetArmAndElevatorEncoderCommand(arm, elevator),
            new DriveForwardGivenDistance(-0.20, driveTrain),
            new PrintCommand("drive back finished"),
            new PlaceCubeOnNode(intake, elevator, arm, ElevatorConstants.CUBE_TOP_HEIGHT, ArmConstants.CUBE_TOP_ANGLE),
            new PrintCommand("place cone on node finished"),
            // leaves community then drives back on charging station
            new DriveForwardGivenDistance(-4.2, driveTrain),
            // wait one second before driving back on charging station so charging station becomes level again
            new WaitCommand(1),
            // drive back on charging station
            new DriveForwardGivenDistance(2.1, driveTrain),
            new PrintCommand("drive forward given distance finished "),

            // balances robot when it's on charging station
            new EngageCmd(driveTrain, 0.2)
          );
    }

}

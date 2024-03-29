package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.robot.PlaceConeOnNode;
import frc.robot.commands.robot.PlaceCubeOnNode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

 // TODO: tune so it stays close to charge station
public class OneCubeCommunity extends SequentialCommandGroup {

    public OneCubeCommunity(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm) {

        addCommands(
            RobotContainer.resetArmAndElevatorEncoderCommand(arm, elevator),
            new DriveForwardGivenDistance(-0.20, driveTrain),
            new PlaceCubeOnNode(intake, elevator, arm, ElevatorConstants.CUBE_TOP_HEIGHT, ArmConstants.CUBE_TOP_ANGLE),
            new PrintCommand("place cone on node finished"),
            // leaves community
            new DriveForwardGivenDistance(-4.2, driveTrain)
            
          );
    }

}

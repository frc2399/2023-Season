package frc.robot.commands.auton;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.commands.robot.PlaceConeOnNode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class OnePieceLeaveEngage extends SequentialCommandGroup {

    public OnePieceLeaveEngage(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm) {

        addCommands(
            new DriveForwardGivenDistance(Units.inchesToMeters(-6.5), driveTrain),
            new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE),
            new PrintCommand("place cone on node finished"),
            
            // drive on charging station
            new DriveForwardGivenDistance(-4, driveTrain),
            new PrintCommand("drive forward given distance finished "),
            new DriveForwardGivenDistance(2, driveTrain),

            // balances robot when it's on charging station
            new EngageCmd(driveTrain)
          );
    }

}

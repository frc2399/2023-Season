package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;

public class LeaveEngage extends SequentialCommandGroup {

    public LeaveEngage(DriveTrain driveTrain, Arm arm, Elevator elevator) {

        addCommands(
            RobotContainer.resetArmAndElevatorEncoderCommand(arm, elevator),
            // leaves community and drives over charging station
            new DriveForwardGivenDistance(-4.2, driveTrain),
            // drives back on charging station
            new DriveForwardGivenDistance(2.1, driveTrain),
             // balances robot when it's on charging station
            new EngageCmd(driveTrain)
          );

    }
}

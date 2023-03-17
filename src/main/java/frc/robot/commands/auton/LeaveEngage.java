package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            new DriveForwardGivenDistance(-4.3, driveTrain),
            // drives back on charging station
            new PrintCommand("finshed driving back"),
            new WaitCommand(1),
            new DriveForwardGivenDistance(2.1, driveTrain),
            new PrintCommand("finshed driving foward"),
             // balances robot when it's on charging station
            new EngageCmd(driveTrain)
          );

    }
}

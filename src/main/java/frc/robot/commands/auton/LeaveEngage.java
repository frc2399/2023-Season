package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class LeaveEngage extends SequentialCommandGroup {

    public LeaveEngage(DriveTrain driveTrain) {

        addCommands(
            // leaves community and drives over charging station
            new DriveForwardGivenDistance(-4, driveTrain),
            // drives back on charging station
            new DriveForwardGivenDistance(2, driveTrain),
             // balances robot when it's on charging station
            new EngageCmd(driveTrain)
          );

    }
}

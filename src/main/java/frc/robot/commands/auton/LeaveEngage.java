package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class LeaveEngage extends SequentialCommandGroup {

    public LeaveEngage(DriveTrain driveTrain) {

        addCommands(
            new DriveForwardGivenDistance(-4, driveTrain),
            new DriveForwardGivenDistance(2, driveTrain),
            new EngageCmd(driveTrain)
          );

    }
}

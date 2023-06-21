package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;

public class ShakingNo extends SequentialCommandGroup {

    public ShakingNo(DriveTrain driveTrain) {

        addCommands(
            new TurnNAngle(0.25, Math.PI/4, driveTrain),
            new TurnNAngle(0.25, -Math.PI/4, driveTrain),
            new TurnNAngle(0.25, Math.PI/4, driveTrain),
            new TurnNAngle(0.25, -Math.PI/4, driveTrain)
          );

    }
}

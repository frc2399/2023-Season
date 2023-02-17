package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.SetArmAngleCmd;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.commands.elevator.SetElevatorPositionCmd;
import frc.robot.commands.intake.IntakeForGivenTime;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class OnePieceEngage extends SequentialCommandGroup {

    public OnePieceEngage(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm) {

        addCommands(
            new ParallelCommandGroup(
                new SetArmAngleCmd(arm),
                new SequentialCommandGroup(
                    new InstantCommand(() -> arm.setTargetAngle(0)),
                    new SetElevatorPositionCmd(elevator, 1),
                    new PrintCommand("arm angle: " + arm.getTargetAngle()),
                    new IntakeForGivenTime(intake, IntakeConstants.INTAKE_OUT_SPEED, 1),
                    new SetElevatorPositionCmd(elevator, ElevatorConstants.MIN_ELEVATOR_HEIGHT)
                )
            ),
            new DriveForwardGivenDistance(0.3, 4, driveTrain),
            new DriveForwardGivenDistance(-0.3, 2, driveTrain),
            new EngageCmd()
          );
    }
}

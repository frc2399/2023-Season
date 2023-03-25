package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.commands.drivetrain.TurnToNAngleCmd;
import frc.robot.commands.intake.IntakeForGivenTime;
import frc.robot.commands.robot.PlaceConeOnNode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class OneAndHalfPieceEngage extends SequentialCommandGroup {

    public OneAndHalfPieceEngage(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm) {

        addCommands(
            new InstantCommand(() -> {driveTrain.resetOdometry(new Pose2d(2.75, 3.26, new Rotation2d(-3.14)));}, driveTrain),
            RobotContainer.resetArmAndElevatorEncoderCommand(arm, elevator),
            new DriveForwardGivenDistance(-0.20, driveTrain),
            new PlaceConeOnNode(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE),
            new PrintCommand("place cone on node finished"),
            // leaves community
            new DriveForwardGivenDistance(-4.2, driveTrain),
            new ParallelCommandGroup(
                // lower arm
                RobotContainer.makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_UP_INTAKE_ANGLE, ElevatorConstants.CONE_UP_INTAKE_HEIGHT),
                new TurnToNAngleCmd(Units.degreesToRadians(-5), driveTrain)
            ),
            // drives and intakes cone off ground
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new DriveForwardGivenDistance(0.63, driveTrain),
                    new RunCommand(() -> driveTrain.setMotors(0.1, 0.1), driveTrain).withTimeout(0.25) 
                ),
                new IntakeForGivenTime(intake, IntakeConstants.CONE_IN_SPEED, 2)),
            new ParallelCommandGroup(
                RobotContainer.makeSetPositionArmAndElevatorCommand(ArmConstants.TURTLE_ANGLE, 0),
                // drive back on charging station
                new DriveForwardGivenDistance(-3.05, driveTrain),
                new IntakeForGivenTime(intake, IntakeConstants.CONE_IN_SPEED, 0.5)
            ),
            new EngageCmd(driveTrain)

            
          );
    }

}

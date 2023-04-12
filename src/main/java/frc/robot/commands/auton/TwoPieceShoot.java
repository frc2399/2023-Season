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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.DriveStraightGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.commands.drivetrain.TurnToNAngleCmd;
import frc.robot.commands.intake.IntakeForGivenTime;
import frc.robot.commands.robot.PlaceConeOnNodeNoTurtle;
import frc.robot.commands.robot.PlaceCubeOnNode;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class TwoPieceShoot extends SequentialCommandGroup {
    
    public TwoPieceShoot(DriveTrain driveTrain, Intake intake, Elevator elevator, Arm arm) {
        double angle1 = 5;
        double angle2 = 180;
        double xpose = 1.88;
        double ypose = 5.06;

        addCommands(
            new InstantCommand(() -> {driveTrain.resetOdometry(new Pose2d(xpose, ypose, new Rotation2d(-3.14)));}, driveTrain),
            RobotContainer.resetArmAndElevatorEncoderCommand(arm, elevator),
            new DriveForwardGivenDistance(-0.20, driveTrain),
            new PlaceConeOnNodeNoTurtle(intake, elevator, arm, ElevatorConstants.CONE_TOP_HEIGHT, ArmConstants.CONE_TOP_ANGLE),
            new PrintCommand("place cone on node finished"),
            new ParallelCommandGroup(
                RobotContainer.makeSetPositionCommand(arm, ArmConstants.TURTLE_ANGLE),
                RobotContainer.makeSetPositionCommand(elevator, 0),
                new WaitUntilCommand(() -> arm.atGoal()),
                new WaitUntilCommand(() -> elevator.atGoal()),
                new DriveStraightGivenDistance(-4.2, 1.5, driveTrain)
            ),
            new ParallelCommandGroup(
                // lower arm
                RobotContainer.makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_UP_INTAKE_ANGLE, ElevatorConstants.CONE_UP_INTAKE_HEIGHT),
                new TurnToNAngleCmd(Units.degreesToRadians(angle1), driveTrain)
            ),
            // // drives and intakes cube off ground
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new DriveForwardGivenDistance(0.4, driveTrain),
                    new RunCommand(() -> driveTrain.setMotors(0.1, 0.1), driveTrain).withTimeout(0.4) 
                ),
            new IntakeForGivenTime(intake, IntakeConstants.CONE_IN_SPEED, 2)),
            // intake cone a bit more while turning
            new ParallelDeadlineGroup(
                new TurnToNAngleCmd(Units.degreesToRadians(angle2), driveTrain),
                RobotContainer.makeSetPositionArmAndElevatorCommand(ArmConstants.TURTLE_ANGLE, 0),
                new IntakeForGivenTime(intake, IntakeConstants.CONE_IN_SPEED, 0.5)
            ),
            // Drive on part of charging station
            new DriveStraightGivenDistance(3, 1.25, driveTrain),
            // shoot cone to score
            new IntakeForGivenTime(intake, IntakeConstants.CONE_OUT_SPEED, 0.5),
            // drive on charging station to engage
            new DriveStraightGivenDistance(1, 1.25, driveTrain),
            // engage
            new EngageCmd(driveTrain, 0.2)
            );
    }
}

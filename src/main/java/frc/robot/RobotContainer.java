package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.CollectPieceCmd;
import frc.robot.commands.IntakeForGivenTime;
import frc.robot.commands.SetElevatorPositionCmd;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.util.DriveTurnControls;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */

public class RobotContainer {

    public final Elevator elevator = new Elevator();
    // The robot's subsystems
    public final static DriveTrain driveTrain = new DriveTrain();
    public final static Intake intake = new Intake();

    // Joysticks
    public static Joystick joystick = new Joystick(JoystickConstants.JOYSTICK_PORT);
    public static Joystick xbox = new Joystick(XboxConstants.XBOX_PORT);

    private DriveTurnControls driveTurnControls = new DriveTurnControls(xbox);
    private Command extendElevator = new SetElevatorPositionCmd(elevator, 1.0);
    private Command middleElevator = new SetElevatorPositionCmd(elevator, .5);
    private Command retractElevator = new SetElevatorPositionCmd(elevator, Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT);
    private Command collectPiece = new CollectPieceCmd(intake);
    private Command dropCone = new InstantCommand(() -> {intake.drop();}, intake);
    private Command bigIntake = new InstantCommand(() -> {intake.intakeBothArms();}, intake);
    private Command leftOnly = new InstantCommand(() -> {intake.intakeLeft();}, intake);
    private Command rightOnly = new InstantCommand(() -> {intake.intakeRight();}, intake);
    private Command noSpin = new InstantCommand(() -> {intake.setMotor(0);}, intake);

    public RobotContainer(){

        DriverStation.silenceJoystickConnectionWarning(true);
    
        // Configure the button bindings
        configureButtonBindings();
    

        // Configure default commands
        driveTrain.setDefaultCommand(
            new ArcadeDriveCmd(driveTrain,
                () -> -driveTurnControls.getDrive(),
                () -> driveTurnControls.getTurn()));
        intake.setDefaultCommand(noSpin);

    }

    private void configureButtonBindings() {
        new JoystickButton(joystick,3).whileTrue(extendElevator);
        new JoystickButton(joystick,4).whileTrue(retractElevator);
        new JoystickButton(joystick,5).whileTrue(middleElevator);
        new JoystickButton(joystick,6).whileTrue(dropCone);
        new JoystickButton(joystick,7).whileTrue(collectPiece);
    }

    public Command getAutonomousCommand() {
        //PathPlannerTrajectory examplePath = PathPlanner.loadPath("Go Straight", new PathConstraints(1, 1));
        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2

        
        //driveTrain.field.getObject("goStraightTrajectory").setTrajectory(goStraight);

        //mirror if on red alliance
        boolean useAllianceColor = true;

        PathPlannerTrajectory twoPiecePath = PathPlanner.loadPath("Two-Cone Auton", new PathConstraints(1, 1));
        driveTrain.field.getObject("traj").setTrajectory(twoPiecePath);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("leftCommunity", new PrintCommand("Left community"));
        eventMap.put("intake", new IntakeForGivenTime(intake, IntakeConstants.INTAKE_IN_SPEED, 2));
        
        Command eventTesting = 
        new SequentialCommandGroup(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                
                driveTrain.resetOdometry(twoPiecePath.getInitialPose());

            }, driveTrain),
        new PPRamseteCommand(
            twoPiecePath,
            () -> driveTrain.getPoseMeters(), // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.DriveConstants.ks,
                Constants.DriveConstants.kv,
                Constants.DriveConstants.ka),
            Constants.DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
            () -> driveTrain.getWheelSpeedsMetersPerSecond(), // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0
                    // will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            (left, right) -> driveTrain.setMotorVoltage(left, right), // voltage
            driveTrain // Requires this drive subsystem
        ));

        FollowPathWithEvents twoPieceAuton = new FollowPathWithEvents(
            eventTesting,
            twoPiecePath.getMarkers(),
            eventMap
        );

        return new SequentialCommandGroup(
            new SetElevatorPositionCmd(elevator, 1.0), 
            new IntakeForGivenTime(intake, IntakeConstants.INTAKE_OUT_SPEED, 1),
            new SetElevatorPositionCmd(elevator, Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT),
            twoPieceAuton,
            new SetElevatorPositionCmd(elevator, 1.0),
            new IntakeForGivenTime(intake, IntakeConstants.INTAKE_OUT_SPEED, 1),
            new SetElevatorPositionCmd(elevator, Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT)
        );
    
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         // Reset odometry for the first path you run during auto
                
        //         driveTrain.resetOdometry(examplePath.getInitialPose());

        //     }, driveTrain),
        //     new PPRamseteCommand(
        //         examplePath,
        //         () -> driveTrain.getPoseMeters(), // Pose supplier
        //         new RamseteController(),
        //         new SimpleMotorFeedforward(Constants.DriveConstants.ks,
        //             Constants.DriveConstants.kv,
        //             Constants.DriveConstants.ka),
        //         Constants.DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
        //         () -> driveTrain.getWheelSpeedsMetersPerSecond(), // DifferentialDriveWheelSpeeds supplier
        //         new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0
        //                 // will only use feedforwards.
        //         new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
        //         (left, right) -> driveTrain.setMotorVoltage(left, right), // voltage
        //         useAllianceColor, 
        //         driveTrain // Requires this drive subsystem
        //     ));

    }

}

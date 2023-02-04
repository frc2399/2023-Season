package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.intake.CollectPieceCmd;
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

    // The robot's subsystems
    public static final DriveTrain driveTrain = new DriveTrain();
    public static final Intake intake = new Intake();
    public static final Elevator elevator = new Elevator();

    // Joysticks
    public static final Joystick joystick = new Joystick(JoystickConstants.JOYSTICK_PORT);
    public static final Joystick xbox = new Joystick(XboxConstants.XBOX_PORT);

    private DriveTurnControls driveTurnControls = new DriveTurnControls(xbox);
    //private Command extendElevator = new SetElevatorPositionCmd(elevator, 1); owo
    //private Command middleElevator = new SetElevatorPositionCmd(elevator, .5);
    //private Command retractElevator = new SetElevatorPositionCmd(elevator, Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT);
    private Command setElevatorSpeedUp = new RunCommand(() -> elevator.setSpeed(0.2), elevator);
    private Command setElevatorSpeedDown = new RunCommand(() -> elevator.setSpeed(-0.2), elevator);
    private Command stopElevator = new InstantCommand(() -> elevator.setSpeed(0), elevator);
    private Command collectPiece = new CollectPieceCmd(intake);
    private Command dropCone = new InstantCommand(() -> intake.drop(), intake);
    // private Command bigIntake = new InstantCommand(() -> intake.intakeBothArms(), intake);
    // private Command leftOnly = new InstantCommand(() -> intake.intakeLeft(), intake);
    // private Command rightOnly = new InstantCommand(() -> intake.intakeRight(), intake);
    private Command noSpin = new RunCommand(() -> intake.setSpeed(0), intake);
    private Command spinIn = new RunCommand(() -> intake.setSpeed(0.4), intake);
    private Command spitOut = new RunCommand(() -> intake.setSpeed(-0.4), intake);

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
        elevator.setDefaultCommand(stopElevator);

    }

    private void configureButtonBindings() {
        new JoystickButton(joystick,3).whileTrue(setElevatorSpeedUp);
        new JoystickButton(joystick,4).whileTrue(setElevatorSpeedDown);
        new JoystickButton(joystick,6).whileTrue(dropCone);
        new JoystickButton(joystick,7).whileTrue(collectPiece);
        new JoystickButton(joystick,8).whileTrue(spinIn);
        new JoystickButton(joystick,9).whileTrue(spitOut);
        new JoystickButton(joystick,11).whileTrue(new InstantCommand(() -> intake.closeRight(), intake));
        new JoystickButton(joystick,12).whileTrue(new InstantCommand(() -> intake.openRight(), intake));
    }

    public Command getAutonomousCommand() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Go Straight", new PathConstraints(1, 1));
        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2

        driveTrain.field.getObject("traj").setTrajectory(examplePath);
        //driveTrain.field.getObject("goStraightTrajectory").setTrajectory(goStraight);

        //mirror if on red alliance
        boolean useAllianceColor = true;

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                
                driveTrain.resetOdometry(examplePath.getInitialPose());

            }, driveTrain),
            new PPRamseteCommand(
                examplePath,
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
                useAllianceColor, 
                driveTrain // Requires this drive subsystem
            ));

    }

}

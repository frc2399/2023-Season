package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.SetElevatorPositionCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
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
    public static MechanismLigament2d elevatorMechanism;
    public static MechanismLigament2d armMechanism;

    // The robot's subsystems
    public final static DriveTrain driveTrain = new DriveTrain();
    public final static Arm arm = new Arm();

    // Joysticks
    public static Joystick joystick = new Joystick(JoystickConstants.JOYSTICK_PORT);
    public static Joystick xbox = new Joystick(XboxConstants.XBOX_PORT);

    private DriveTurnControls driveTurnControls = new DriveTurnControls(xbox);
    private Command extendElevator = new SetElevatorPositionCmd(elevator, 1);
    private Command middleElevator = new SetElevatorPositionCmd(elevator, .5);
    private Command retractElevator = new SetElevatorPositionCmd(elevator, Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT);
   // private Command moveArm = new SetElevatorPositionCmd(elevator, Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT);
    private Command moveArmUp = new RunCommand(() -> {arm.setSpeed(1);}, arm);
    private Command moveArmDown = new RunCommand(() -> {arm.setSpeed(-1);}, arm);
    private Command armDefaultCmd = new InstantCommand(() -> {arm.setSpeed(0);}, arm);


    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();
        arm.setDefaultCommand(armDefaultCmd);

        // Configure default commands
        driveTrain.setDefaultCommand(
            new ArcadeDriveCmd(driveTrain,
                () -> -driveTurnControls.getDrive(),
                () -> driveTurnControls.getTurn()));
        
        Mechanism2d mech = new Mechanism2d(3, 2);
        MechanismRoot2d root = mech.getRoot("root", 2, 0);
        elevatorMechanism = root.append(new MechanismLigament2d("elevator", Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT, 50));
        armMechanism = elevatorMechanism.append(new MechanismLigament2d("arm", Constants.ArmConstants.ARM_LENGTH, 90));
        SmartDashboard.putData("Mech2d", mech);

    }

    private void configureButtonBindings() {
        new JoystickButton(joystick,3).whenPressed(extendElevator);
        new JoystickButton(joystick,4).whenPressed(retractElevator);
        new JoystickButton(joystick,5).whenPressed(middleElevator);
        new JoystickButton(joystick,6).whileTrue(moveArmUp);
        new JoystickButton(joystick, 7).whileTrue(moveArmDown);

    }

    public Command getAutonomousCommand() {
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));

        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2

        driveTrain.field.getObject("traj").setTrajectory(examplePath);

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
                () -> driveTrain.getWheelSpeeds(), // DifferentialDriveWheelSpeeds supplier
                new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0
                        // will only use feedforwards.
                new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
                (left, right) -> driveTrain.setMotors(left, right), // power biconsumer
                driveTrain // Requires this drive subsystem
            ));

    }

}

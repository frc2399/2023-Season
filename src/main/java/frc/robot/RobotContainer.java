package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.subsystems.DriveTrain;
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
    public final static DriveTrain driveTrain = new DriveTrain();

    // Joysticks
    public static Joystick joystick = new Joystick(JoystickConstants.JOYSTICK_PORT);
    public static Joystick xbox = new Joystick(XboxConstants.XBOX_PORT);

    private DriveTurnControls driveTurnControls = new DriveTurnControls(xbox);

    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        driveTrain.setDefaultCommand(
            new ArcadeDriveCmd(driveTrain,
                () -> -driveTurnControls.getDrive(),
                () -> driveTurnControls.getTurn()));

    }

    private void configureButtonBindings() {
        // Drive train

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

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
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
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
       return new SequentialCommandGroup(
           new PrintCommand("Sequential Command Group Started"),
           new PrintCommand("Print Command Ran"),
            
            new PrintCommand("Auton Finish"));
    //     //This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    //     return new SequentialCommandGroup(
    //         new PrintCommand("Start of Auton Command"),
    //         new InstantCommand(() -> {
    //           // Reset odometry for the first path you run during auto
    //           System.out.println("Before Resetting Odometry");
    //            //   driveTrain.resetOdometry(examplePath.getInitialPose());
    
    //         }, driveTrain ),
    //         new PrintCommand("After Instant Command"),
    //         new PPRamseteCommand(
    //             examplePath, 
    //             () -> driveTrain.getPoseMeters(), // Pose supplier
    //             new RamseteController(),
    //             new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
    //             Constants.DriveConstants.kDriveKinematics, // DifferentialDriveKinematics
    //             () -> driveTrain.getWheelSpeeds(), // DifferentialDriveWheelSpeeds supplier
    //             new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //             new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
    //             (left, right) -> driveTrain.tankDriveVolts(left, right), // Voltage biconsumer
    //             driveTrain // Requires this drive subsystem
    //         ),
    //         new PrintCommand("End of Auton Command")
    //     );

     }

   
}


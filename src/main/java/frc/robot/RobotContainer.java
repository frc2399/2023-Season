package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    public final static Shifter shifter = new Shifter();

    // Joysticks
    public static Joystick joystick = new Joystick(JoystickConstants.JOYSTICK_PORT);
    public static Joystick xbox = new Joystick(XboxConstants.XBOX_PORT);

    private DriveTurnControls driveTurnControls = new DriveTurnControls(xbox);

    // Defining commands
    private static InstantCommand shiftHighTorque = new InstantCommand(() -> shifter.setShifterHighTorque(),
        shifter);
    private static InstantCommand shiftHighSpeed = new InstantCommand(() -> shifter.setShifterHighSpeed(), shifter);

    private RobotContainer() {

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
        new JoystickButton(xbox, XboxController.Button.kLeftBumper.value).whenPressed(shiftHighTorque);
        new JoystickButton(xbox, XboxController.Button.kRightBumper.value).whenPressed(shiftHighSpeed);
        

    }

}

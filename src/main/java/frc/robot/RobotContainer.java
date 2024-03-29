package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.auton.Engage;
import frc.robot.commands.auton.LeaveEngage;
import frc.robot.commands.auton.OneAndHalfConeBump;
import frc.robot.commands.auton.OneAndHalfConeEngage;
import frc.robot.commands.auton.OneAndHalfConeNoEngage;
import frc.robot.commands.auton.OneAndHalfCubeBump;
import frc.robot.commands.auton.OneAndHalfCubeNoEngage;
import frc.robot.commands.auton.OneCubeCommunity;
import frc.robot.commands.auton.OneCubeCommunityEngage;
import frc.robot.commands.auton.OneAndHalfCubeEngage;
import frc.robot.commands.auton.OnePieceCommunity;
import frc.robot.commands.auton.OnePieceCommunityEngage;
import frc.robot.commands.auton.OnePieceDoNothing;
import frc.robot.commands.auton.OnePieceEngage;
import frc.robot.commands.auton.TwoPieceAuton;
import frc.robot.commands.auton.TwoPieceAutonBump;
import frc.robot.commands.drivetrain.CurvatureDriveCmd;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.commands.intake.IntakeForGivenTime;
import frc.robot.commands.intake.StallIntakeCmd;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.RealArm;
import frc.robot.subsystems.arm.SimArm;
import frc.robot.subsystems.drivetrain.DriveIO;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.RealDrive;
import frc.robot.subsystems.drivetrain.SimDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.RealElevator;
import frc.robot.subsystems.elevator.SimElevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.RealIntake;
import frc.robot.subsystems.intake.SimIntake;
import frc.robot.util.DriveUtil;


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
    public static DriveTrain driveTrain;
    public static LED led = new LED();
    public static Arm arm;
   // public static Camera camera;

    public static Intake intake;
    public static Elevator elevator;
    // public static PoseEstimator poseEstimator;
    // public static PhotonCamera photonCamera;

    public static MechanismLigament2d elevatorMechanism;
    public static MechanismLigament2d armMechanism;
    public static MechanismLigament2d LEDMechanism;

    // Joysticks
    public static final Joystick xboxDriver = new Joystick(XboxConstants.XBOX_DRIVER_PORT);
    public static final Joystick xboxOperator = new Joystick(XboxConstants.XBOX_OPERATOR_PORT);

    public static PowerDistribution pdp;

    public static boolean coneMode = true;

    private Command changeMode;

    private Command setTopPieceSetpoint;
    private Command setMidPieceSetpoint;
    private Command setLowPieceSetpoint;
    private Command setGroundUpIntakeSetpoint;
    private Command setGroundTipIntakeSetpoint;
    private Command setShelfIntakeSetpoint;

    private Command selectPositionCommand;

    private Command turtleMode;

    // so the enum is not initialized to dangerous position
    public static CommandSelector angleHeight = CommandSelector.CONE_GROUND_INTAKE;

     // A chooser for autonomous commands
     final SendableChooser<Command> chooser = new SendableChooser<>();
     final ComplexWidget autonChooser = Shuffleboard.getTab("Driver")
     .add("Choose Auton", chooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(4, 4).withSize(9, 1);

    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);
        pdp = new PowerDistribution(1, ModuleType.kRev);
        // photonCamera = new PhotonCamera ("photonvision");

        // camera not in simulator to make it not crash
        // if (RobotBase.isReal()) {
        //     CameraServer.startAutomaticCapture();
        // }

        setUpSubsystems();

        // poseEstimator = new PoseEstimator(photonCamera, driveTrain);

        setUpAutonChooser();
        setUpConeCubeCommands();
        configureButtonBindings();
        setDefaultCommands();
        simulationMechanisms();
    }

    private void configureButtonBindings() {

        // Operator Left Bumper (5) - changes from cone to cube mode (intially cone mode, shows on smartdashboard)
        new JoystickButton(xboxOperator, Button.kLeftBumper.value).onTrue(changeMode);

        // Driver Left Stick (9) - change from normal to slow mode
        new JoystickButton(xboxDriver, Button.kLeftStick.value).onTrue(new InstantCommand(() -> {CurvatureDriveCmd.isSlow = !CurvatureDriveCmd.isSlow;}));

        // Operator Right Y Axis (5) - moves arm up at 0.2 speed, moves arm down at 0.2 speed
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kRightY.value) < -0.1).whileTrue(makeSetSpeedGravityCompensationCommand(arm, 0.2)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kRightY.value) > 0.1).whileTrue(makeSetSpeedGravityCompensationCommand(arm, -0.2)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));

        // Driver Button A (1) - resets arm encoder position to intial offset (at the top)
        new JoystickButton(xboxDriver, Button.kA.value).onTrue(resetArmEncoderCommand(arm));

        // Operator Left Y Axis (1) - moves elevator up at 0.2 speed, moves elevator down at 0.4 speed
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kLeftY.value) < -0.1).whileTrue(makeSetSpeedGravityCompensationCommand(elevator, 0.2)).onFalse(makeSetSpeedGravityCompensationCommand(elevator, 0));
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kLeftY.value) > 0.1).whileTrue(makeSetSpeedGravityCompensationCommand(elevator, -0.4)).onFalse(makeSetSpeedGravityCompensationCommand(elevator, 0));

        // Driver Button B (2) - resets elevator encoder to intial offset (at the bottom)
        new JoystickButton(xboxDriver, Button.kB.value).onTrue(resetElevatorEncoderCommand(elevator));

        // Operator Button A (1) - sets the arm and elevator setpoints for the low node
        new JoystickButton(xboxOperator, Button.kA.value).onTrue(setLowPieceSetpoint);

        // Operator Button X (3) - sets the arm and elevator setpoints for the mid node
        new JoystickButton(xboxOperator, Button.kX.value).onTrue(setMidPieceSetpoint);

        // Operator Button Y (4) - sets the arm and elevator setpoints for the top node
        new JoystickButton(xboxOperator, Button.kY.value).onTrue(setTopPieceSetpoint);

        // Operator Right Trigger Axis (3) - sends arm and elevator to selected setpoint
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kRightTrigger.value) > 0.1).whileTrue(selectPositionCommand);

        // Operator Right Bupmper
        new JoystickButton(xboxOperator, Button.kRightBumper.value).onTrue(setGroundUpIntakeSetpoint);

        // Operator Left Trigger Axis (2) - sends the arm and elevator to the positions for intaking cones from the tip on the ground and cubes
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kLeftTrigger.value) > 0.1).whileTrue(turtleMode);

        // Currently no binding to `setGroundTipIntakeSetpoint`
        // new Trigger(() -> xboxOperator.getRawAxis(Axis.kLeftTrigger.value) > 0.1).whileTrue(setGroundTipIntakeSetpoint);

        // Operator Button B (2) - sends the arm and elevator to the positions for intaking pieces from the shelf
        new JoystickButton(xboxOperator, Button.kB.value).onTrue(setShelfIntakeSetpoint);

        //Kill command - sets speeds of subsystems to 0

        // Driver Right Bumper - sends arm and elevator to selected setpoint
        new JoystickButton(xboxDriver, Button.kRightBumper.value).onTrue(selectPositionCommand);

        // Driver Left Bumper - robot goes into turtle mode (arm all the  way up, elevator all the way down)
        new JoystickButton(xboxDriver, Button.kLeftBumper.value).onTrue(turtleMode);

        // Operator Right Bumper (6) - kill command (sets speeds of subsystems to 0)
        // new JoystickButton(xboxOperator,Button.kRightBumper.value).whileTrue(new InstantCommand(() -> {
        //     makeSetSpeedGravityCompensationCommand(elevator, 0);
        //     makeSetSpeedGravityCompensationCommand(arm, 0);
        //     intake.setMotor(0);
        // }, elevator, arm, intake, driveTrain));

        // Driver X(3) - engage command
        new JoystickButton(xboxDriver, Button.kX.value).whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> {CurvatureDriveCmd.isSlow = true;}),
                new EngageCmd(driveTrain, 0.3)
        ));

        //Unused Buttons
            //Driver -
            //Operator -

        //Turn to angle button
        // new JoystickButton(xboxDriver, Button.kRightStick.value).onTrue(new TurnToNAngleCmd(Math.PI, driveTrain));

        //intake for given time button
        new JoystickButton(xboxDriver, 8).onTrue(new IntakeForGivenTime(intake, IntakeConstants.CUBE_IN_SPEED, 1.5));

    }

    private void setDefaultCommands() {
        // Classic curvature
        // driveTrain.setDefaultCommand(
        //     new CurvatureDriveCmd(driveTrain,
        //         () -> xboxDriver.getRawAxis(XboxController.Axis.kLeftY.value),
        //         () -> xboxDriver.getRawAxis(XboxController.Axis.kRightX.value),
        //         () -> elevator.getEncoderPosition()));


        // Tank Drive
        // driveTrain.setDefaultCommand(
        //         new RunCommand(
        //             () -> {
        //                 driveTrain.setMotors(
        //                     DriveUtil.computeDeadband(-1*xboxDriver.getRawAxis(XboxController.Axis.kLeftY.value), 0.05),
        //                     DriveUtil.computeDeadband(-1*xboxDriver.getRawAxis(XboxController.Axis.kRightY.value), 0.05));
        //             }, driveTrain));


        // Cleaner curvature
        driveTrain.setDefaultCommand(
            new RunCommand(() -> {
                var speed = -xboxDriver.getRawAxis(XboxController.Axis.kLeftY.value);
                var turn_cw = xboxDriver.getRawAxis(XboxController.Axis.kRightX.value);

                speed = speed * speed * speed;
                turn_cw = turn_cw * turn_cw * turn_cw;

                speed = MathUtil.applyDeadband(speed, 0.05);
                turn_cw = MathUtil.applyDeadband(turn_cw, 0.05);

                // Turn proportional to fwd speed, unless fwd speed is too small in which case we still want to be able to turn
                turn_cw *= Math.max(Math.abs(speed), 0.6);

                var speed_left = speed + turn_cw;
                var speed_right = speed -turn_cw;
                var max_output = List.of(speed_left, speed_right).stream().max((a, b) -> (int) (a - b)).orElse(1.0);
                if (max_output > 1) {
                    speed_left /= max_output;
                    speed_right /= max_output;
                }

                if (CurvatureDriveCmd.isSlow || elevator.getEncoderPosition() > ElevatorConstants.MAX_ELEVATOR_HEIGHT / 2) {
                    speed_left *= DriveConstants.SLOW_SPEED_FRACTION;
                    speed_right *= DriveConstants.SLOW_SPEED_FRACTION;
                }

                driveTrain.setMotors(speed_left, speed_right);
            }, driveTrain)
        );

        //right d-pad to intake on operator
        intake.setDefaultCommand(
            new StallIntakeCmd(intake,
            //right d-pad to intake on operator
            () -> (xboxDriver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.1 || xboxOperator.getPOV() == 90),
            //allows left d-pad to outtake
            () -> xboxDriver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.1 || xboxOperator.getPOV() == 270));
        // elevator.setDefaultCommand(new InstantCommand(() -> elevator.setSpeed(0), elevator));
        // arm.setDefaultCommand(new SetArmAngleCmd(arm));

    }

    private void simulationMechanisms() {
        //Makes a mechanism (lines to show elevator and arm) in simulator
        //Team colors!
        Mechanism2d mech = new Mechanism2d(1, 1);
        MechanismRoot2d root = mech.getRoot("root", 0.3, 0);
        elevatorMechanism = root.append(new MechanismLigament2d("elevator", Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT, 50));
        elevatorMechanism.setColor(new Color8Bit(0, 204, 255));
        elevatorMechanism.setLineWeight(20);
        armMechanism = elevatorMechanism.append(new MechanismLigament2d("arm", Constants.ArmConstants.ARM_LENGTH, 140));
        armMechanism.setColor(new Color8Bit(200, 0, 216));
        LEDMechanism = root.append(new MechanismLigament2d("LED", 0.1, 0));
        LEDMechanism.setColor(new Color8Bit(0, 0, 0));
        LEDMechanism.setLineWeight(20);
        SmartDashboard.putData("Mech2d", mech);
    }

    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        String autonCommand = chooser.getSelected().toString();
        DataLogManager.log("Auton: " + autonCommand);
        return chooser.getSelected();
    }

    private void setUpConeCubeCommands () {
        changeMode = new InstantCommand(() -> {coneMode = !coneMode;});

        //UIUtil.setRumblePattern(3, xboxOperator);
        setTopPieceSetpoint = new InstantCommand(() -> {
                angleHeight = coneMode ? CommandSelector.CONE_TOP : CommandSelector.CUBE_TOP;
            });

        //UIUtil.setRumblePattern(2, xboxOperator);
        setMidPieceSetpoint = new InstantCommand(() -> {
                angleHeight = coneMode ? CommandSelector.CONE_MID : CommandSelector.CUBE_MID;
            });

        //UIUtil.setRumblePattern(1, xboxOperator);
        setLowPieceSetpoint = new InstantCommand(() -> {
                angleHeight = coneMode ? CommandSelector.CONE_LOW : CommandSelector.CUBE_LOW;
            });

        setGroundUpIntakeSetpoint = new InstantCommand(() -> {
                angleHeight = coneMode ? CommandSelector.CONE_GROUND_INTAKE : CommandSelector.CUBE_INTAKE;
            });

        setGroundTipIntakeSetpoint = new InstantCommand(() -> {
                angleHeight = coneMode ? CommandSelector.CONE_TIP_INTAKE : CommandSelector.CUBE_INTAKE;
            });

        setShelfIntakeSetpoint = new InstantCommand(() -> {
                angleHeight = coneMode ? CommandSelector.CONE_SHELF : CommandSelector.CUBE_SHELF;
            });

        selectPositionCommand = selectPositionCommand();

        // coneTipIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_TIP_INTAKE_ANGLE, ElevatorConstants.CONE_TIP_INTAKE_HEIGHT);
        // conePhalangeIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_PHALANGE_INTAKE_ANGLE, ElevatorConstants.CONE_PHALANGE_INTAKE_HEIGHT);
        turtleMode = makeSetPositionArmAndElevatorCommand(ArmConstants.TURTLE_ANGLE, 0.0);
    }

    private void setUpSubsystems () {

        DriveIO driveIO;
        ElevatorIO elevatorIO;
        ArmIO armIO;
        IntakeIO intakeIO;
        // implemented driveIO interface
        if (RobotBase.isSimulation()) {
            driveIO = new SimDrive();
            elevatorIO = new SimElevator();
            armIO = new SimArm();
            intakeIO = new SimIntake();
        } else {
            driveIO = new RealDrive();
            elevatorIO = new RealElevator();
            armIO = new RealArm();
            intakeIO = new RealIntake();
        }

        driveTrain = new DriveTrain(driveIO);
        elevator = new Elevator(elevatorIO);
        arm = new Arm(armIO);
        intake = new Intake(intakeIO);
        //limelight = new SimLimelight(driveTrain);
        // camera = new Camera(photonCamera);

    }

    private void setUpAutonChooser () {
        chooser.setDefaultOption("do nothing", new PrintCommand("i am doing nothing"));
        chooser.addOption("engage", new Engage(driveTrain, arm, elevator));
        chooser.addOption("leave and engage", new LeaveEngage(driveTrain, arm, elevator));
        chooser.addOption("score and do nothing", new OnePieceDoNothing(driveTrain, intake, elevator, arm));
        chooser.addOption("score and engage", new OnePieceEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("score, leave community, and engage", new OnePieceCommunityEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("score and leave community", new OnePieceCommunity(driveTrain, intake, elevator, arm));
        chooser.addOption("leave community", new DriveForwardGivenDistance(-5, driveTrain));
        chooser.addOption("one and half cone and engage", new OneAndHalfConeEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("one and half cube and engage", new OneAndHalfCubeEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("one and half cube NO engage", new OneAndHalfCubeNoEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("one and half cone NO engage", new OneAndHalfConeNoEngage(driveTrain, intake, elevator, arm));
        // chooser.addOption("two cone auton bottom", new TwoPieceAutonBottom(driveTrain, elevator, intake, arm));
        chooser.addOption("two piece auton", new TwoPieceAuton(driveTrain, intake, elevator, arm));
        chooser.addOption("one and half cube bump", new OneAndHalfCubeBump(driveTrain, intake, elevator, arm));
        chooser.addOption("one and half cone bump", new OneAndHalfConeBump(driveTrain, intake, elevator, arm));
        chooser.addOption("two piece bump", new TwoPieceAutonBump(driveTrain, intake, elevator, arm));
        chooser.addOption("one cube, leave community, and engage ", new OneCubeCommunityEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("one cube, leave community", new OneCubeCommunity(driveTrain, intake, elevator, arm));



    }

    public static Command makeSetPositionCommand(ProfiledPIDSubsystem base, double target) {
        return new SequentialCommandGroup(
            new ConditionalCommand(new InstantCommand(() -> {}), new InstantCommand(() -> base.enable()), () -> base.isEnabled()),
            new InstantCommand(() -> base.setGoal(target), base)
        );
    }

    private Command makeSetSpeedGravityCompensationCommand(Arm a, double speed) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> a.disable()),
            new RunCommand(() -> a.setSpeedGravityCompensation(speed), a)
        );
    }

    private Command makeSetSpeedGravityCompensationCommand(Elevator e, double speed) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> e.disable()),
            new RunCommand(() -> e.setSpeedGravityCompensation(speed), e)
        );
    }

    private static Command resetArmEncoderCommand(Arm a) {
        Debouncer debouncer = new Debouncer(0.2);
        return new SequentialCommandGroup(
            new PrintCommand("Resetting arm encoder"),
            new InstantCommand(() ->  a.disable()),
            new RunCommand(() -> a.setSpeed(0.15)).withTimeout(0.2),
            new RunCommand(() -> a.setSpeed(0.15)).until(() -> debouncer.calculate(Math.abs(a.getEncoderSpeed()) < 0.01)),
            new InstantCommand(() -> a.setPosition(Constants.ArmConstants.INITIAL_OFFSET)),
            makeSetPositionCommand(a, ArmConstants.TURTLE_ANGLE)
        );
    }

    private static Command resetElevatorEncoderCommand(Elevator e) {
        Debouncer debouncer = new Debouncer(0.2);
        return new SequentialCommandGroup(
            new PrintCommand("Resetting elevator encoder"),
            new InstantCommand(() ->  e.disable()),
            new RunCommand(() -> e.setSpeed(-0.10)).withTimeout(0.2),
            new RunCommand(() -> e.setSpeed(-0.10)).until(() -> debouncer.calculate(Math.abs(e.getEncoderSpeed()) < 0.01)),
            new InstantCommand(() -> e.setPosition(0)),
            makeSetPositionCommand(e, 0)
        );
    }

    public static Command resetArmAndElevatorEncoderCommand(Arm a, Elevator e) {
        return new ParallelCommandGroup(resetArmEncoderCommand(arm),
        resetElevatorEncoderCommand(elevator)
     );
    }

    public static Command makeSetPositionArmAndElevatorCommand(double angle, double height) {
        return new ParallelCommandGroup(
            makeSetPositionCommand(arm, angle),
            makeSetPositionCommand(elevator, height)
        );
    }

    public enum CommandSelector {
        CONE_TOP,
        CONE_MID,
        CONE_LOW,
        CUBE_TOP,
        CUBE_MID,
        CUBE_LOW,
        CONE_SHELF,
        CUBE_SHELF,
        CONE_GROUND_INTAKE,
        CUBE_INTAKE,
        CONE_TIP_INTAKE
      }

    private CommandSelector select() {
        return angleHeight;
    }

    public static String toString(CommandSelector node) {
        return "Node: " + node;
    }

    private Command selectPositionCommand() {
        return new SelectCommand(
            Map.ofEntries(
                Map.entry(CommandSelector.CONE_LOW, makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_LOW_ANGLE, ElevatorConstants.CONE_LOW_HEIGHT)),
                Map.entry(CommandSelector.CUBE_LOW, makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_LOW_ANGLE, ElevatorConstants.CUBE_LOW_HEIGHT)),
                Map.entry(CommandSelector.CONE_MID, makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_MID_ANGLE, ElevatorConstants.CONE_MID_HEIGHT)),
                Map.entry(CommandSelector.CUBE_MID, makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_MID_ANGLE, ElevatorConstants.CUBE_MID_HEIGHT)),
                Map.entry(CommandSelector.CONE_TOP, makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_TOP_ANGLE, ElevatorConstants.CONE_TOP_HEIGHT)),
                Map.entry(CommandSelector.CUBE_TOP, makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_TOP_ANGLE, ElevatorConstants.CUBE_TOP_HEIGHT)),
                Map.entry(CommandSelector.CONE_SHELF,  makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_SHELF_INTAKE_ANGLE, ElevatorConstants.CONE_SHELF_INTAKE_HEIGHT)),
                Map.entry(CommandSelector.CUBE_SHELF,  makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_SHELF_INTAKE_ANGLE, ElevatorConstants.CUBE_SHELF_INTAKE_HEIGHT)),
                Map.entry(CommandSelector.CONE_GROUND_INTAKE, makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_UP_INTAKE_ANGLE, ElevatorConstants.CONE_UP_INTAKE_HEIGHT)),
                Map.entry(CommandSelector.CUBE_INTAKE, makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_INTAKE_ANGLE, ElevatorConstants.CUBE_INTAKE_HEIGHT)),
                Map.entry(CommandSelector.CONE_TIP_INTAKE, makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_TIP_INTAKE_ANGLE, ElevatorConstants.CONE_TIP_INTAKE_HEIGHT))),
            this::select);
        }
}

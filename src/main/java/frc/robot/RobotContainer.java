package frc.robot;

import org.photonvision.PhotonCamera;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.CameraAimCmd;
import frc.robot.commands.auton.Engage;
import frc.robot.commands.auton.LeaveEngage;
import frc.robot.commands.auton.OnePieceCommunityEngage;
import frc.robot.commands.auton.OnePieceEngage;
import frc.robot.commands.auton.TwoPieceAuton;
import frc.robot.commands.drivetrain.CurvatureDriveCmd;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.commands.drivetrain.TurnToNAngleCmd;
import frc.robot.commands.auton.TwoPieceAutonBottom;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.CurvatureDriveCmd;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
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
import frc.robot.subsystems.limelight.Camera;
import frc.robot.subsystems.limelight.PoseEstimator;
import frc.robot.subsystems.limelight.SimLimelight;


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
    public static SimLimelight limelight;
    public static Camera camera;
    
    public static Intake intake;
    public static Elevator elevator;
    public static PoseEstimator poseEstimator;
    public static PhotonCamera photonCamera;
     
    public static MechanismLigament2d elevatorMechanism;
    public static MechanismLigament2d armMechanism;
    public static MechanismLigament2d LEDMechanism;

    // Joysticks
    public static final Joystick xboxDriver = new Joystick(XboxConstants.XBOX_DRIVER_PORT);
    public static final Joystick xboxOperator = new Joystick(XboxConstants.XBOX_OPERATOR_PORT);

    public static boolean coneMode = true;

    private Command coneIntakeShelf;
    private Command cubeIntakeShelf; 
    private Command coneUprightIntakePosition;
    private Command coneTipIntakePosition;
    private Command conePhalangeIntakePosition;
    private Command cubeIntakePosition;
    private Command coneIntake;
    private Command cubeIntake;
    private Command coneOutake;
    private Command cubeOutake;
    
    private Command changeMode;

    private Command chaseTagCmd;

    private Command setTopPieceSetpoint;
    private Command setMidPieceSetpoint;
    private Command setLowPieceSetpoint;
    private Command intakeUprightPosition;
    private Command intakePiece;
    private Command outakePiece;
    private Command intakePieceShelf;

    private Command selectScoringPositionCommand;

    private Command turtleMode;


    CommandSelector angleHeight = CommandSelector.CONE_TOP;
    
     // A chooser for autonomous commands
     final SendableChooser<Command> chooser = new SendableChooser<>();
     final ComplexWidget autonChooser = Shuffleboard.getTab("Driver")
     .add("Choose Auton", chooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(4, 4).withSize(9, 1);

    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);

        photonCamera = new PhotonCamera ("photonvision");

        // Configure the button bindings
        
        setUpSubsystems();

        poseEstimator = new PoseEstimator(photonCamera, driveTrain);

        setUpAutonChooser();
        setUpConeCubeCommands();
        configureButtonBindings();
        setDefaultCommands();
        simulationMechanisms();
        setUpDriveCommands();
    }

    private void configureButtonBindings() {
       
        new JoystickButton(xboxOperator, Button.kLeftBumper.value).onTrue(changeMode);

        new JoystickButton(xboxDriver, Button.kLeftStick.value).onTrue(new InstantCommand(() -> {CurvatureDriveCmd.isSlow = !CurvatureDriveCmd.isSlow;}));
        
        // arm up, arm down, reset encoder position to 0
        // new Trigger(() -> xboxOperator.getRawAxis(Axis.kRightY.value) < -0.1).whileTrue(makeSetSpeedGravityCompensationCommand(arm, 0.15)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));
        // new Trigger(() -> xboxOperator.getRawAxis(Axis.kRightY.value) > 0.1).whileTrue(makeSetSpeedGravityCompensationCommand(arm, -0.15)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kRightY.value) < -0.1).whileTrue(makeSetSpeedGravityCompensationCommand(arm, 0.15)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kRightY.value) > 0.1).whileTrue(makeSetSpeedGravityCompensationCommand(arm, -0.15)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));
        //new JoystickButton(xboxDriver,Button.kA.value).whileTrue(new InstantCommand(() -> arm.setPosition(Constants.ArmConstants.INITIAL_OFFSET)));
        // temp
        new JoystickButton(xboxDriver, Button.kA.value).onTrue(resetArmEncoderCommand(arm));

        new JoystickButton(xboxDriver, Button.kX.value).onTrue(new EngageCmd(driveTrain));

        
        new JoystickButton(xboxOperator, Button.kA.value).onTrue(makeSetPositionCommand(arm, 0.8));
        new JoystickButton(xboxOperator, Button.kB.value).onTrue(makeSetPositionCommand(arm, -0.3));

        // elevator up, elevator down, reset encoder position to 0, top preset, mid preset, low preset
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kLeftY.value) < -0.1).whileTrue(makeSetSpeedGravityCompensationCommand(elevator, 0.2)).onFalse(makeSetSpeedGravityCompensationCommand(elevator, 0));
        new Trigger(() -> xboxOperator.getRawAxis(Axis.kLeftY.value) > 0.1).whileTrue(makeSetSpeedGravityCompensationCommand(elevator, -0.2)).onFalse(makeSetSpeedGravityCompensationCommand(elevator, 0));
        //new JoystickButton(xboxDriver,Button.kB.value).whileTrue(new InstantCommand(() -> elevator.setPosition(0)));
        // temp
        new JoystickButton(xboxDriver, Button.kB.value).onTrue(resetElevatorEncoderCommand(elevator));
        
        // new JoystickButton(xboxOperator, Button.kY.value).onTrue(setTopPieceSetpoint);

        //TODO make this work pls lol
        //new JoystickButton(xboxOperator, Button.kY.value).onTrue(CameraAimCmd(driveTrain, photonCamera));
        
        // new JoystickButton(xboxOperator, Button.kX.value).onTrue(setMidPieceSetpoint);
        // new JoystickButton(xboxOperator, Button.kA.value).onTrue(setLowPieceSetpoint);
        
        //intake positions
        //new Trigger(() -> xboxOperator.getRawAxis(Axis.kRightTrigger.value) > 0.1).whileTrue(intakeUprightPosition);
        // new JoystickButton(xboxOperator, Button.kB.value).onTrue(intakePieceShelf);

        //intake commands
        new Trigger(() -> xboxDriver.getRawAxis(Axis.kRightTrigger.value) > 0.1).whileTrue(intakePiece);
        new Trigger(() -> xboxDriver.getRawAxis(Axis.kLeftTrigger.value) > 0.1).whileTrue(outakePiece);
        
        // new JoystickButton(xboxDriver, Button.kLeftBumper.value).onTrue(selectScoringPositionCommand);

        // new JoystickButton(xboxDriver, Button.kRightBumper.value).onTrue(turtleMode);

        //Kill command - sets speeds of subsystems to 0
        new JoystickButton(xboxOperator,Button.kRightBumper.value).whileTrue(new InstantCommand(() -> {
            makeSetSpeedGravityCompensationCommand(elevator, 0);
            makeSetSpeedGravityCompensationCommand(arm, 0);
            intake.setMotor(0);
            driveTrain.setMotors(0, 0);
        }, elevator, arm, intake, driveTrain));
    }

    private void setDefaultCommands() {
        driveTrain.setDefaultCommand(
            new CurvatureDriveCmd(driveTrain,
                () -> xboxDriver.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> xboxDriver.getRawAxis(XboxController.Axis.kRightX.value), () -> elevator.getEncoderPosition()));        

        intake.setDefaultCommand(new RunCommand(() -> intake.setMotor(0), intake));
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
        System.out.println("Autonomous command! " + chooser.getSelected());
        return chooser.getSelected();
    }

    private void setUpConeCubeCommands () {

        coneIntakeShelf = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_SHELF_INTAKE_ANGLE, ElevatorConstants.CONE_SHELF_INTAKE_HEIGHT);
        cubeIntakeShelf = makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_SHELF_INTAKE_ANGLE, ElevatorConstants.CUBE_SHELF_INTAKE_HEIGHT);

        turtleMode = makeSetPositionArmAndElevatorCommand(ArmConstants.INITIAL_OFFSET, ElevatorConstants.MIN_ELEVATOR_HEIGHT);

        coneUprightIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_UP_INTAKE_ANGLE, ElevatorConstants.CONE_UP_INTAKE_HEIGHT);
        // cubeIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_INTAKE_ANGLE, ElevatorConstants.CUBE_INTAKE_HEIGHT);
        // coneTipIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_TIP_INTAKE_ANGLE, ElevatorConstants.CONE_TIP_INTAKE_HEIGHT);
        // conePhalangeIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_PHALANGE_INTAKE_ANGLE, ElevatorConstants.CONE_PHALANGE_INTAKE_HEIGHT);

        coneIntake = new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.CONE_IN_SPEED), intake);
        cubeIntake = new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.CUBE_IN_SPEED), intake);
        coneOutake = new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.CONE_OUT_SPEED), intake);
        cubeOutake = new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.CUBE_OUT_SPEED), intake);

        changeMode = new InstantCommand(() -> {coneMode = !coneMode;});

        setTopPieceSetpoint = new ConditionalCommand(new InstantCommand(() -> angleHeight = CommandSelector.CONE_TOP), new InstantCommand(() -> angleHeight = CommandSelector.CUBE_TOP), () -> coneMode);
        setMidPieceSetpoint = new ConditionalCommand(new InstantCommand(() -> angleHeight = CommandSelector.CONE_MID), new InstantCommand(() -> angleHeight = CommandSelector.CUBE_MID), () -> coneMode);
        setLowPieceSetpoint = new ConditionalCommand(new InstantCommand(() -> angleHeight = CommandSelector.CONE_LOW), new InstantCommand(() -> angleHeight = CommandSelector.CUBE_LOW), () -> coneMode);
        // intakeUprightPosition = new ConditionalCommand(coneUprightIntakePosition, cubeIntakePosition, () -> coneMode);
        intakePiece = new ConditionalCommand(coneIntake, cubeIntake, () -> coneMode);
        outakePiece = new ConditionalCommand(coneOutake, cubeOutake, () -> coneMode);

        intakePieceShelf = new ConditionalCommand(coneIntakeShelf, cubeIntakeShelf, () -> coneMode);

        selectScoringPositionCommand = selectScoringPositionCommand();
    }

    private void setUpSubsystems () {

        DriveIO driveIO;
        ElevatorIO elevatorIO;
        ArmIO armIO;
        IntakeIO intakeIO;
        // implemented drivio interface 
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
        limelight = new SimLimelight(driveTrain);
        camera = new Camera(photonCamera);

    }

    private void setUpAutonChooser () {
        chooser.addOption("two cone auton", new TwoPieceAuton(driveTrain, elevator, intake, arm));
        chooser.addOption("engage", new Engage(driveTrain));
        chooser.addOption("leave and engage", new LeaveEngage(driveTrain));
        chooser.addOption("score and engage", new OnePieceEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("Leave community and engage", new OnePieceCommunityEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("do nothing", new PrintCommand("i am doing nothing"));
        chooser.addOption("leave community", new DriveForwardGivenDistance(-5, driveTrain));
        chooser.addOption("two cone auton bottom", new TwoPieceAutonBottom(driveTrain, elevator, intake, arm));
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

    private Command resetArmEncoderCommand(Arm a) {
        return new SequentialCommandGroup(
            //new InstantCommand(() -> a.setSpeed(0.15)).until(() -> a.getArmCurrent() > Constants.NEO_CURRENT_LIMIT - 5),
            new InstantCommand(() -> a.setPosition(Constants.ArmConstants.INITIAL_OFFSET))
        );
    }
    
    private Command resetElevatorEncoderCommand(Elevator e) {
        return new SequentialCommandGroup(
            //new InstantCommand(() -> e.setSpeed(0.15)).until(() -> e.getElevatorCurrent() > Constants.NEO_CURRENT_LIMIT - 5),
            new InstantCommand(() -> e.setPosition(0))
        );
    }

    public static Command makeSetPositionArmAndElevatorCommand(double angle, double height) {
        return new ParallelCommandGroup(
            makeSetPositionCommand(arm, angle),
            makeSetPositionCommand(elevator, height)
        );
    }

    private void setUpDriveCommands() {
        SmartDashboard.putData("ArcadeDrive",  new ArcadeDriveCmd(driveTrain,
        () -> xboxDriver.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> xboxDriver.getRawAxis(XboxController.Axis.kRightX.value)));
        SmartDashboard.putData("CurvatureDrive",  new CurvatureDriveCmd(driveTrain,
        () -> xboxDriver.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> xboxDriver.getRawAxis(XboxController.Axis.kRightX.value), () -> elevator.getEncoderPosition()));
    }

    private enum CommandSelector {
        CONE_TOP,
        CONE_MID,
        CONE_LOW,
        CUBE_TOP,
        CUBE_MID,
        CUBE_LOW,
      }

    private CommandSelector select() {
        return angleHeight;
    }
        
    private Command selectScoringPositionCommand() {
        return new SelectCommand(
            Map.ofEntries(
                Map.entry(CommandSelector.CONE_LOW, makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_LOW_ANGLE, ElevatorConstants.CONE_LOW_HEIGHT)),
                Map.entry(CommandSelector.CUBE_LOW, makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_LOW_ANGLE, ElevatorConstants.CUBE_LOW_HEIGHT)),
                Map.entry(CommandSelector.CONE_MID, makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_MID_ANGLE, ElevatorConstants.CONE_MID_HEIGHT)),
                Map.entry(CommandSelector.CUBE_MID, makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_MID_ANGLE, ElevatorConstants.CUBE_MID_HEIGHT)),
                Map.entry(CommandSelector.CONE_TOP, makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_TOP_ANGLE, ElevatorConstants.CONE_TOP_HEIGHT)),
                Map.entry(CommandSelector.CUBE_TOP, makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_TOP_ANGLE, ElevatorConstants.CUBE_TOP_HEIGHT))),
            this::select); 
    }
}

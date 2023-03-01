package frc.robot;

import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.auton.Engage;
import frc.robot.commands.auton.LeaveEngage;
import frc.robot.commands.auton.OnePieceEngage;
import frc.robot.commands.auton.TwoPieceAuton;
import frc.robot.commands.auton.TwoPieceAutonBottom;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.CurvatureDriveCmd;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.robot.PlaceConeOnNode;
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
import frc.robot.commands.drivetrain.ArcadeDriveCmd;


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
    
    public static Intake intake;
    public static Elevator elevator;
    
    public static MechanismLigament2d elevatorMechanism;
    public static MechanismLigament2d armMechanism;
    public static MechanismLigament2d LEDMechanism;

    // Joysticks
    public static final Joystick joystick = new Joystick(JoystickConstants.JOYSTICK_PORT);
    public static final Joystick xbox = new Joystick(XboxConstants.XBOX_PORT);

    public static boolean coneMode = true;

    private Command coneTopNode;
    private Command cubeTopNode;
    private Command coneMidNode;
    private Command cubeMidNode;
    private Command coneLowNode;
    private Command cubeLowNode;
    private Command coneUprightIntakePosition;
    private Command coneTipIntakePosition;
    private Command conePhalangeIntakePosition;
    private Command cubeIntakePosition;
    private Command coneIntake;
    private Command cubeIntake;
    private Command coneOutake;
    private Command cubeOutake;
    
    private Command changeMode;

    private Command placePieceTop;
    private Command placePieceMid;
    private Command placePieceLow;
    private Command intakeUprightPosition;
    private Command intakePiece;
    private Command outakePiece;
    
     // A chooser for autonomous commands
     final SendableChooser < Command > chooser = new SendableChooser < > ();
     final ComplexWidget autonChooser = Shuffleboard.getTab("Driver")
     .add("Choose Auton", chooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(4, 4).withSize(9, 1);

    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);
        
        setUpSubsystems();
        setUpAutonChooser();
        setUpConeCubeCommands();
        configureButtonBindings();
        setDefaultCommands();
        simulationMechanisms();
        setUpDriveCommands();
    }

    private void configureButtonBindings() {
       
        new JoystickButton(xbox, Button.kA.value).onTrue(changeMode);
        // new JoystickButton(xbox, Button.kA.value).onTrue(new InstantCommand(() -> {coneMode = true;}));
        // new JoystickButton(xbox, Button.kB.value).onTrue(new InstantCommand(() -> {coneMode = false;}));

        new JoystickButton(joystick, 2).onTrue(new InstantCommand(() -> {ArcadeDriveCmd.isSlow = !ArcadeDriveCmd.isSlow;}));
        
        // arm up, arm down, reset encoder position to 0 (move the arm all the way up then hit)
        new JoystickButton(joystick,4).whileTrue(makeSetSpeedGravityCompensationCommand(arm, 0.15)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));
        new JoystickButton(joystick,6).whileTrue(makeSetSpeedGravityCompensationCommand(arm, -0.15)).onFalse(makeSetSpeedGravityCompensationCommand(arm, 0));
        new JoystickButton(joystick,11).whileTrue(new InstantCommand(() -> arm.setPosition(Constants.ArmConstants.INITIAL_OFFSET)));

        // elevator up, elevator down, reset encoder position to 0 (move elevator all the way down then hit), top preset, mid preset, low preset
        new JoystickButton(joystick,3).whileTrue(makeSetSpeedGravityCompensationCommand(elevator, 0.2)).onFalse(makeSetSpeedGravityCompensationCommand(elevator, 0));
        new JoystickButton(joystick,5).whileTrue(makeSetSpeedGravityCompensationCommand(elevator, -0.2)).onFalse(makeSetSpeedGravityCompensationCommand(elevator, 0));
        new JoystickButton(joystick,12).whileTrue(new InstantCommand(() -> elevator.setPosition(0)));
        new JoystickButton(xbox, Button.kY.value).onTrue(placePieceTop);
        new JoystickButton(xbox, Button.kX.value).onTrue(placePieceMid);
        new JoystickButton(xbox, Button.kB.value).onTrue(placePieceLow);
        
        //positions to intake upright cone from ground
        new JoystickButton(joystick,1).onTrue(makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_UP_INTAKE_ANGLE, ElevatorConstants.CONE_UP_INTAKE_HEIGHT));
        // new JoystickButton(joystick,1).onTrue(intakeUprightPosition);
        // new JoystickButton(xbox, Button.kRightBumper.value).onTrue(conePhalangeIntakePosition);
        // new JoystickButton(xbox, Button.kLeftBumper.value).onTrue(coneTipIntakePosition);
        
        //intake commands
        new Trigger(() -> xbox.getRawAxis(Axis.kLeftTrigger.value) > 0.1).whileTrue(intakePiece);
        new Trigger(() -> xbox.getRawAxis(Axis.kRightTrigger.value) > 0.1).whileTrue(outakePiece);

        //TODO make proper kill command :O
        // new JoystickButton(joystick,5).whileTrue(new InstantCommand(() -> elevator.setSpeed(0), elevator));
    }

    private void setDefaultCommands() {
        driveTrain.setDefaultCommand(
            new CurvatureDriveCmd(driveTrain,
                () -> xbox.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> xbox.getRawAxis(XboxController.Axis.kRightX.value)));        

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

        coneTopNode = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_TOP_ANGLE, ElevatorConstants.CONE_TOP_HEIGHT);
        cubeTopNode = makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_TOP_ANGLE, ElevatorConstants.CUBE_TOP_HEIGHT);
        coneMidNode = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_MID_ANGLE, ElevatorConstants.CONE_MID_HEIGHT);
        cubeMidNode = makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_MID_ANGLE, ElevatorConstants.CUBE_MID_HEIGHT);
        coneLowNode = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_LOW_ANGLE, ElevatorConstants.CONE_LOW_HEIGHT);
        cubeLowNode = makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_LOW_ANGLE, ElevatorConstants.CUBE_LOW_HEIGHT);

        coneUprightIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_UP_INTAKE_ANGLE, ElevatorConstants.CONE_UP_INTAKE_HEIGHT);
        // cubeIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CUBE_INTAKE_ANGLE, ElevatorConstants.CUBE_INTAKE_HEIGHT);
        // coneTipIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_TIP_INTAKE_ANGLE, ElevatorConstants.CONE_TIP_INTAKE_HEIGHT);
        // conePhalangeIntakePosition = makeSetPositionArmAndElevatorCommand(ArmConstants.CONE_PHALANGE_INTAKE_ANGLE, ElevatorConstants.CONE_PHALANGE_INTAKE_HEIGHT);

        coneIntake = new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.CONE_IN_SPEED), intake);
        cubeIntake = new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.CUBE_IN_SPEED), intake);
        coneOutake = new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.CONE_OUT_SPEED), intake);
        cubeOutake = new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.CUBE_OUT_SPEED), intake);

        changeMode = new InstantCommand(() -> {coneMode = !coneMode;});

        placePieceTop = new ConditionalCommand(coneTopNode, cubeTopNode, () -> coneMode);
        placePieceMid = new ConditionalCommand(coneMidNode, cubeMidNode, () -> coneMode);
        placePieceLow = new ConditionalCommand(coneLowNode, cubeLowNode, () -> coneMode);
        // intakeUprightPosition = new ConditionalCommand(coneUprightIntakePosition, cubeIntakePosition, () -> coneMode);
        intakePiece = new ConditionalCommand(coneIntake, cubeIntake, () -> coneMode);
        outakePiece = new ConditionalCommand(coneOutake, cubeOutake, () -> coneMode);
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

    }

    private void setUpAutonChooser () {
        chooser.addOption("two cone auton", new TwoPieceAuton(driveTrain, elevator, intake, arm));
        chooser.addOption("engage", new Engage(driveTrain));
        chooser.addOption("leave and engage", new LeaveEngage(driveTrain));
        chooser.addOption("score and engage", new OnePieceEngage(driveTrain, intake, elevator, arm));
        chooser.addOption("do nothing", new PrintCommand("i am doing nothing"));
        chooser.addOption("leave community", new DriveForwardGivenDistance(-1, 5, driveTrain));
        chooser.addOption("two cone auton bottom", new TwoPieceAutonBottom(driveTrain, elevator, intake, arm));
    }  
    
    public static Command makeSetPositionCommand(ProfiledPIDSubsystem base, double target) {
        return new SequentialCommandGroup(
            new ConditionalCommand(new InstantCommand(() -> {}), new InstantCommand(() -> base.enable()), () -> base.isEnabled()),    
            new InstantCommand(() -> base.setGoal(target), base)
        );
    }

    private Command makeSetSpeedGravityCompensationCommand(Arm a, double target) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> a.disable()), 
            new RunCommand(() -> a.setSpeedGravityCompensation(target), a)
        );
    }

    private Command makeSetSpeedGravityCompensationCommand(Elevator e, double target) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> e.disable()), 
            new RunCommand(() -> e.setSpeedGravityCompensation(target), e)
        );
    }

    private Command makeSetPositionArmAndElevatorCommand(double angle, double height) {
        return new ParallelCommandGroup(
            makeSetPositionCommand(arm, angle),
            makeSetPositionCommand(elevator, height)
        );
    }

    private void setUpDriveCommands() {
        SmartDashboard.putData("ArcadeDrive",  new ArcadeDriveCmd(driveTrain,
        () -> xbox.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> xbox.getRawAxis(XboxController.Axis.kRightX.value)));
        SmartDashboard.putData("CurvatureDrive",  new CurvatureDriveCmd(driveTrain,
        () -> xbox.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> xbox.getRawAxis(XboxController.Axis.kRightX.value)));
    }

}
 
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.auton.Engage;
import frc.robot.commands.auton.LeaveEngage;
import frc.robot.commands.auton.OnePieceEngage;
import frc.robot.commands.auton.TwoPieceAuton;
import frc.robot.commands.drivetrain.ArcadeDriveCmd;
import frc.robot.commands.drivetrain.DriveForwardGivenDistance;
import frc.robot.commands.drivetrain.EngageCmd;
import frc.robot.commands.elevator.SetElevatorPositionCmd;
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
    private Command changeMode;

    // private Command changeToConeMode;
    // private Command changeToCubeMode;

    private Command placePieceTop;
    private Command placePieceMid;
    private Command placePieceLow;
    
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

    }

    private void configureButtonBindings() {
       
        new JoystickButton(xbox, Button.kA.value).onTrue(changeMode);
       
        // new JoystickButton(xbox,XboxMappingToJoystick.A_BUTTON).onTrue(new InstantCommand(() -> {coneMode = true;}));
        // new JoystickButton(xbox,XboxMappingToJoystick.B_BUTTON).onTrue(new InstantCommand(() -> {coneMode = false;}));

        // set elevator to bottom 
        new JoystickButton(xbox, Button.kY.value).onTrue(new SetElevatorPositionCmd(elevator, Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT));

        new JoystickButton(joystick,12).whileTrue(new InstantCommand(() -> {arm.setTargetAngle(Math.PI/4);}));
        new JoystickButton(joystick, 13).whileTrue(new InstantCommand(() -> {arm.setTargetAngle(-Math.PI/4 * 3);}));
        new JoystickButton(joystick, 2).whileTrue(new InstantCommand(() -> {arm.setTargetAngle(-Math.PI/4);}));
        new JoystickButton(joystick,3).whileTrue(new RunCommand(() -> elevator.setSpeed(0.2), elevator));
        new JoystickButton(joystick,4).whileTrue(new RunCommand(() -> elevator.setSpeed(-0.2), elevator));
        //TODO make proper kill command :O
        new JoystickButton(joystick,5).whileTrue(new InstantCommand(() -> elevator.setSpeed(0), elevator));
        // new JoystickButton(joystick,6).whileTrue(new InstantCommand(() -> intake.drop(), intake));
        // new JoystickButton(joystick,7).whileTrue(new CollectPieceCmd(intake));
        // new JoystickButton(xbox,XboxMappingToJoystick.A_BUTTON).onTrue(changeToConeMode);
        // new JoystickButton(xbox,XboxMappingToJoystick.B_BUTTON).onTrue(changeToCubeMode);

        // if coneMode true, set elevator to cone mode for top node
        new JoystickButton(xbox, Button.kX.value).onTrue(placePieceTop);
        
        // new JoystickButton(joystick,7).whileTrue(collectPiece);
        new JoystickButton(joystick,8).whileTrue(new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.INTAKE_IN_SPEED), intake));
        new JoystickButton(joystick,9).whileTrue(new RunCommand(() -> intake.setMotor(Constants.IntakeConstants.INTAKE_OUT_SPEED), intake));
        
    }

    private void setDefaultCommands() {
        driveTrain.setDefaultCommand(
            new ArcadeDriveCmd(driveTrain,
                () -> xbox.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> xbox.getRawAxis(XboxController.Axis.kRightX.value)));        

        intake.setDefaultCommand(new RunCommand(() -> intake.setMotor(0), intake));
        elevator.setDefaultCommand(new InstantCommand(() -> elevator.setSpeed(0), elevator));
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

        coneTopNode = new SetElevatorPositionCmd(elevator, ElevatorConstants.CONE_TOP_NODE_HEIGHT);
        cubeTopNode = new SetElevatorPositionCmd(elevator, ElevatorConstants.CUBE_TOP_NODE_HEIGHT);
        coneMidNode = new SetElevatorPositionCmd(elevator, ElevatorConstants.CONE_MID_NODE_HEIGHT);
        cubeMidNode = new SetElevatorPositionCmd(elevator, ElevatorConstants.CUBE_MID_NODE_HEIGHT);
        coneLowNode = new SetElevatorPositionCmd(elevator, ElevatorConstants.CONE_LOW_NODE_HEIGHT);
        cubeLowNode = new SetElevatorPositionCmd(elevator, ElevatorConstants.CUBE_LOW_NODE_HEIGHT);

        changeMode = new InstantCommand(() -> {coneMode = !coneMode;});

        
        placePieceTop = new ConditionalCommand(coneTopNode, cubeTopNode, () -> coneMode);
        placePieceMid = new ConditionalCommand(coneMidNode, cubeMidNode, () -> coneMode);
        placePieceLow = new ConditionalCommand(coneLowNode, cubeLowNode, () -> coneMode);
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
    }    
}

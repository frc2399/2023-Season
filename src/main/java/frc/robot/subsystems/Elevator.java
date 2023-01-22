package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SimEncoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class Elevator extends SubsystemBase {
  
  private CANSparkMax leftMotorController;
  private CANSparkMax rightMotorController;
  private RelativeEncoder leftEncoder, rightEncoder;
  public static SimEncoder elevatorEncoderSim;
  private ElevatorSim elevatorSim;
    public static final double ELEVATOR_KP = 0;//1.875;
  public static final double ELEVATOR_KI = 0;//0.006;
  public static final double ELEVATOR_KD = 0;//52.5;
  public static final double ELEVATOR_KF = 0.000086; //0.15;
  public static final double ELEVATOR_KIZ = 0;
  public static final double ELEVATOR_K_MAX_OUTPUT = 1;
  public static final double ELEVATOR_K_MIN_OUTPUT = 0;
  public static final double ELEVATOR_MAX_RPM = 5700;

  // Encoder, motor controller, and joystick to be simulated
  private final Encoder encoder = new Encoder(0, 1);
  public final static PWMSparkMax motorController = new PWMSparkMax(0);
  public final static Joystick joystick = new Joystick(0);

  
  // Simulated elevator constants and gearbox
  private static final double elevatorGearRatio = 50.0;
  private static final double elevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double elevatorCarriageMass = 4.0; // kg

  private static final double minElevatorHeight = Units.inchesToMeters(2);
  private static final double maxElevatorHeight = Units.inchesToMeters(75);

  private final DCMotor elevatorGearbox = DCMotor.getVex775Pro(2);


  //smart dashboard stuff
  public static final GenericEntry elevatorPos = Shuffleboard.getTab("Meow").addPersistent("Elevator Position", 0).getEntry();
  public static final GenericEntry elevatorVel = Shuffleboard.getTab("Meow").addPersistent("Elevator Velocity", 0).getEntry();


  // The simulated encoder will return 
  public static final double elevatorEncoderDistPerPulse =
      2.0 * Math.PI * elevatorDrumRadius / 4096;

  public Elevator() {
    //initialize motor controllers
    leftMotorController = new CANSparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    rightMotorController = new CANSparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    //restore factory settings to reset to a known state
    leftMotorController.restoreFactoryDefaults();
    rightMotorController.restoreFactoryDefaults();

    //set climber motors to coast mode
    leftMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //initialize motor encoder
    leftEncoder = leftMotorController.getEncoder();
    rightEncoder = rightMotorController.getEncoder();

    //invert the motor controllers so climber climbs right
    leftMotorController.setInverted(false);
    rightMotorController.setInverted(false);
    
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // In addition, we can put useful information on the smart dashboard for graphing
    SmartDashboard.putNumber("Elevator Position", elevatorEncoderSim.getDistance());
    SmartDashboard.putNumber("Elevator Velocity", elevatorEncoderSim.getSpeed());


    //this code is instantiating the simulator stuff for climber
    if(RobotBase.isSimulation()) {
        elevatorEncoderSim = new SimEncoder("Elevator");
        elevatorSim = new ElevatorSim(
          DCMotor.getNEO(1), //1 NEO motor on the climber
          elevatorGearRatio,
          elevatorCarriageMass, 
          elevatorDrumRadius,
          minElevatorHeight,
          maxElevatorHeight,
          false,
          VecBuilder.fill(0.01)
        ); 

        leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
        rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION); }
    }

    @Override
  public void simulationPeriodic() {
    //sets input for elevator motor in simulation
    elevatorSim.setInput(leftMotorController.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
    // elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
    //sets our simulated encoder speeds
    elevatorEncoderSim.setSpeed(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    
  }

     public double getElevatorHeight()
    {
    return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
    }

    public double getLeftEncoderPosition()
  {
    if (RobotBase.isSimulation())
    {
      // simulator output is in meters, needs to be converted to inches to work with the rest of the code. encoders are already in inches
        return Units.metersToInches(elevatorEncoderSim.getDistance());
    }
    else
    {
        //gets position in inches
        return leftEncoder.getPosition();
    }
  }

  public double getRightEncoderPosition()
  {
    if (RobotBase.isSimulation())
      // simulator output is in meters, needs to be converted to inches to work with the rest of the code. encoders are already in inches
    {
        return Units.metersToInches(elevatorEncoderSim.getDistance());
    }
    else
    {
        //gets position in inches
        return rightEncoder.getPosition();
    }
  }
  

}


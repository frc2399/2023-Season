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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  
  private CANSparkMax elevatorMotorController;
  private RelativeEncoder elevatorEncoder;
  public static EncoderSim elevatorEncoderSim;
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
  public final static Joystick elevatorJoystick = new Joystick(0);


  public final DoublePublisher elevatorPosition;
  public final DoublePublisher elevatorVelocity;
  
  // Simulated elevator constants and gearbox
  private static final double elevatorGearRatio = 50.0;
  private static final double elevatorDrumRadius = Units.inchesToMeters(2.0);
  private static final double elevatorCarriageMass = 4.0; // kg

  private static final double minElevatorHeight = Units.inchesToMeters(2);
  private static final double maxElevatorHeight = Units.inchesToMeters(75);

  private final DCMotor elevatorGearbox = DCMotor.getVex775Pro(2);



  SlewRateLimiter filter;



  // The simulated encoder will return 
  public static final double elevatorEncoderDistPerPulse =
      2.0 * Math.PI * elevatorDrumRadius / 4096;

  public Elevator() {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
     // get the subtable called "datatable"
     NetworkTable datatable = inst.getTable("datatable");

      // publish to the topic in "datatable" called "Out"
    elevatorPosition = datatable.getDoubleTopic("elevator Pos").publish();
    elevatorVelocity = datatable.getDoubleTopic("elevator Vel").publish();

    //initialize motor controllers
    elevatorMotorController = new CANSparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  

    //restore factory settings to reset to a known state
    elevatorMotorController.restoreFactoryDefaults();


    //set climber motors to coast mode
    elevatorMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
  

    //initialize motor encoder
    elevatorEncoder = elevatorMotorController.getEncoder();
  

    //invert the motor controllers so climber climbs right
    elevatorMotorController.setInverted(false);
    

   filter = new SlewRateLimiter(SmartDashboard.getNumber("Climber Slew Rate", ElevatorConstants.ELEVATOR_SLEW));


    elevatorEncoder.setPosition(0);
  



  
    //this code is instantiating the simulator stuff for climber
    if(RobotBase.isSimulation()) {
      Encoder encoder = new Encoder(2,3);
        elevatorEncoderSim = new EncoderSim(encoder);
        // final Joystick elevatorJoystick = new Joystick(0);
        elevatorSim = new ElevatorSim(
          elevatorGearbox,
          elevatorGearRatio,
          elevatorCarriageMass, 
          elevatorDrumRadius,
          minElevatorHeight,
          maxElevatorHeight,
          false,
          VecBuilder.fill(0.01)

        ); 

        elevatorEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
        
    }
      
    }

    @Override
  public void simulationPeriodic() {

    elevatorPosition.set( elevatorEncoderSim.getDistance());
    elevatorVelocity.set( elevatorEncoderSim.getRate());
    //sets input for elevator motor in simulation
    elevatorSim.setInput(motorController.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
    // elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
    //sets our simulated encoder speeds
    elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    

  
  }

     public double getElevatorHeight()
    {
    return (elevatorEncoder.getPosition());
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
        return elevatorEncoder.getPosition();
    }
  }

  
  
  public void setSpeed(double speed)
  {
    speed = filter.calculate(speed);
    elevatorMotorController.set(speed);
  }


}


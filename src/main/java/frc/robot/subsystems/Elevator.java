package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.SimEncoder;

public class Elevator extends SubsystemBase {

  public static CANSparkMax elevatorMotorController;
  public static RelativeEncoder elevatorEncoder;
  public static SimEncoder elevatorSimEncoder;
  public static ElevatorSim elevatorSim;

  public final DoublePublisher elevatorPositionPublisher;
  public final DoublePublisher elevatorVelocityPublisher;

  // Simulated elevator constants and gearbox
  public static final double elevatorGearRatio = 50.0;
  public static final double elevatorDrumRadius = Units.inchesToMeters(2.0);
  public static final double elevatorCarriageMass = 4.0; // kg

  public static final double minElevatorHeight = Units.inchesToMeters(2);
  public static final double maxElevatorHeight = Units.inchesToMeters(75);

  public final static DCMotor elevatorGearbox = DCMotor.getNEO(1);

  private static double current_pos = 0;
  private static double current_vel = 0;

  // The simulated encoder will return
  public static final double elevatorEncoderDistPerPulse = 2.0 * Math.PI * elevatorDrumRadius / 4096;

  public Elevator() {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("datatable");

    // publish to the topic in "datatable" called "Out"
    elevatorPositionPublisher = datatable.getDoubleTopic("elevator Pos").publish();
    elevatorVelocityPublisher = datatable.getDoubleTopic("elevator Vel").publish();

    // initialize motor controllers
    elevatorMotorController = new CANSparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);

    // restore factory settings to reset to a known state
    elevatorMotorController.restoreFactoryDefaults();

    // set climber motors to coast mode
    elevatorMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // initialize motor encoder
    elevatorEncoder = elevatorMotorController.getEncoder();

    // invert the motor controllers so climber climbs right
    elevatorMotorController.setInverted(false);

    elevatorEncoder.setPosition(0); //do we need this??? owo

    // this code is instantiating the simulator stuff for climber
    if (RobotBase.isSimulation()) {
      elevatorSimEncoder = new SimEncoder("elevator");
      // final Joystick elevatorJoystick = new Joystick(0);
      elevatorSim = new ElevatorSim(
          elevatorGearbox,
          elevatorGearRatio,
          elevatorCarriageMass,
          elevatorDrumRadius,
          minElevatorHeight,
          maxElevatorHeight,
          true,
          VecBuilder.fill(0.01)
      );
    }
  }

  @Override
  public void simulationPeriodic() {

    current_pos = elevatorSimEncoder.getDistance();
    current_vel = elevatorSimEncoder.getSpeed();
    elevatorPositionPublisher.set(current_pos);
    elevatorVelocityPublisher.set(current_vel);

    // sets input for elevator motor in simulation
    elevatorSim.setInput(elevatorMotorController.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    elevatorSimEncoder.setDistance(elevatorSim.getPositionMeters());
    // elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
    // sets our simulated encoder speeds
    elevatorSimEncoder.setSpeed(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

  }

  // public double getElevatorHeight() {
  // return (elevatorEncoder.getPosition());
  // }

  //returns height the elevator is at
  public double getEncoderPosition() {
    if (RobotBase.isSimulation()) {
      // simulator output is in meters, needs to be converted to inches to work with
      // the rest of the code. encoders are already in inches
      return elevatorSimEncoder.getDistance();
    }
    else
    {
      return elevatorEncoder.getPosition();
    }
  }

  //returns speed of elevator
  public double getEncoderSpeed() {
    if (RobotBase.isSimulation()) {
      // simulator output is in meters, needs to be converted to inches to work with
      // the rest of the code. encoders are already in inches
      return elevatorSimEncoder.getSpeed();
    }
    else
    {
      return elevatorEncoder.getVelocity();
    }
  }



  // else {
  // // gets position in inches
  // // return elevatorEncoder.getPosition();
  // }
  // }

  public void setSpeed(double speed) {
    elevatorMotorController.set(speed);
  }
  


}

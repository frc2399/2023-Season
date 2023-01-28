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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  public static CANSparkMax elevatorMotorController;
  public static RelativeEncoder elevatorEncoder;
  public static EncoderSim elevatorEncoderSim;
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

  public static double elevatorSetpoint;
  public static DoublePublisher elevatorSetpointPublisher;
  public static DoublePublisher elevatorTargetPosPublisher;
  public static DoublePublisher elevatorTargetVelPublisher;

  // private static final double feedForward = 0.55;
  // private static final double kpPos = 1;
  // private static final double kpVel = 0.01;

  // tuned values:
  private static final double kpFeedForward = 0.55;
  private static final double kpPos = 2;
  private static final double kpVel = .2;

  private static double current_pos = 0;
  private static double current_vel = 0;
  private static double gravityCompensation = .075;

  // Trapezoidal profile constants and variables
  private static final double max_vel = 1.0; // m/s
  private static final double max_accel = 1.0; // m/s/s

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(max_vel, max_accel);
  TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State(0.0, 0.0);

  // The simulated encoder will return
  public static final double elevatorEncoderDistPerPulse = 2.0 * Math.PI * elevatorDrumRadius / 4096;

  public Elevator() {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("datatable");

    // publish to the topic in "datatable" called "Out"
    elevatorPositionPublisher = datatable.getDoubleTopic("elevator Pos").publish();
    elevatorVelocityPublisher = datatable.getDoubleTopic("elevator Vel").publish();

    elevatorSetpointPublisher = datatable.getDoubleTopic("elevator setpoint").publish();
    elevatorTargetPosPublisher = datatable.getDoubleTopic("Target Pos").publish();
    elevatorTargetVelPublisher = datatable.getDoubleTopic("Target Vel").publish();

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

    // elevatorEncoder.setPosition(0);

    // this code is instantiating the simulator stuff for climber
    if (RobotBase.isSimulation()) {
      Encoder encoder = new Encoder(2, 3);
      elevatorEncoderSim = new EncoderSim(encoder);
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

    elevatorPositionPublisher.set(elevatorEncoderSim.getDistance());
    elevatorVelocityPublisher.set(elevatorEncoderSim.getRate());
    current_pos = elevatorEncoderSim.getDistance();
    current_vel = elevatorEncoderSim.getRate();
    elevatorSetpointPublisher.set(elevatorSetpoint);
    // sets input for elevator motor in simulation
    elevatorSim.setInput(elevatorMotorController.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
    // elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
    // sets our simulated encoder speeds
    elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);

    // elevatorSetpoint);

    // Update the profile each timestep to get the current target position and
    // velocity
    TrapezoidProfile.State referenceSetpoint = new TrapezoidProfile.State(elevatorSetpoint, 0.0);
    TrapezoidProfile profile = new TrapezoidProfile(constraints, referenceSetpoint, previousProfiledReference);
    previousProfiledReference = profile.calculate(0.02); // Assumes 50Hz loop. Could measure this directly

    double target_pos = previousProfiledReference.position;
    double target_vel = previousProfiledReference.velocity;

    elevatorTargetPosPublisher.set(target_pos);
    elevatorTargetVelPublisher.set(target_vel);

    double speed = kpFeedForward * target_vel + kpPos * (target_pos - current_pos) + kpVel * (target_vel - current_vel);
    speed = speed + gravityCompensation;
    elevatorMotorController.set(speed);
  }

  // public double getElevatorHeight() {
  // return (elevatorEncoder.getPosition());
  // }

  //returns height the elevator is at
  public double getEncoderPosition() {
    if (RobotBase.isSimulation()) {
      // simulator output is in meters, needs to be converted to inches to work with
      // the rest of the code. encoders are already in inches
      return Units.metersToInches(elevatorEncoderSim.getDistance());
    }
    else
    {
      return elevatorEncoder.getPosition();
    }
  }

  //returns speed of elevator
  public double getEncoderRate() {
    if (RobotBase.isSimulation()) {
      // simulator output is in meters, needs to be converted to inches to work with
      // the rest of the code. encoders are already in inches
      return Units.metersToInches(elevatorEncoderSim.getRate());
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

    //elevatorEncoderSim.setInput(Elevator.elevatorMotorController.get() * RobotController.getBatteryVoltage());
  }
  
  


}

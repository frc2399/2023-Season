package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.SimEncoder;

public class Elevator extends SubsystemBase {

  public static CANSparkMax elevatorMotorController;
  public static RelativeEncoder elevatorEncoder;
  public static SimEncoder elevatorSimEncoder;
  public static ElevatorSim elevatorSim;
  private MechanismLigament2d elevatorMechanism;

  // Simulated elevator constants and gearbox
  public static final double elevatorGearRatio = 50.0;
  public static final double elevatorDrumRadius = Units.inchesToMeters(2.0);
  public static final double elevatorCarriageMass = 4.0; // kg


  public final static DCMotor elevatorGearbox = DCMotor.getNEO(1);

  private static double currentPos = 0;
  private static double currentVel = 0;

  // The simulated encoder will return
  public static final double elevatorEncoderDistPerPulse = 2.0 * Math.PI * elevatorDrumRadius / 4096;

  public Elevator() {

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
      elevatorSim = new ElevatorSim(
          elevatorGearbox,
          elevatorGearRatio,
          elevatorCarriageMass,
          elevatorDrumRadius,
          Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT,
          Constants.ElevatorConstants.MAX_ELEVATOR_HEIGHT,
          true,
          VecBuilder.fill(0.01)
      );
      Mechanism2d mech = new Mechanism2d(3, 2);
      MechanismRoot2d root = mech.getRoot("root", 2, 0);
      elevatorMechanism = root.append(new MechanismLigament2d("elevator", Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT, 50));
      SmartDashboard.putData("Mech2d", mech);
    }
  }

  @Override
  public void simulationPeriodic() {

    currentPos = elevatorSimEncoder.getDistance();
    currentVel = elevatorSimEncoder.getSpeed();
    SmartDashboard.putNumber("elevator position", currentPos); 
    SmartDashboard.putNumber("elevator velocity", currentVel); 

    // sets input for elevator motor in simulation
    elevatorSim.setInput(elevatorMotorController.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    elevatorSimEncoder.setDistance(elevatorSim.getPositionMeters());
    // sets our simulated encoder speeds
    elevatorSimEncoder.setSpeed(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    elevatorMechanism.setLength(Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT + currentPos);

  }

  //returns height the elevator is at
  public double getEncoderPosition() {
    if (RobotBase.isSimulation()) {
      // simulator output is in meters, needs to be converted to inches to work with
      // the rest of the code. encoders are already in inches
      return elevatorSimEncoder.getDistance();
    }
    else {
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
    else {
      return elevatorEncoder.getVelocity();
    }
  }

  public void setSpeed(double speed) {
    elevatorMotorController.set(speed);
  }
  
}

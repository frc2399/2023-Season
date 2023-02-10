package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.SimEncoder;

public class SimElevator implements ElevatorIO {
    public static SimEncoder elevatorSimEncoder;
    public static ElevatorSim elevatorSim;
    public double elevatorPower;
    private MechanismLigament2d elevatorMechanism;
    public static CANSparkMax elevatorMotorControllerRight;
    public final static DCMotor elevatorGearbox = DCMotor.getNEO(1);
    // Simulated elevator constants and gearbox
    public static final double elevatorGearRatio = 50.0;
    public static final double elevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double elevatorCarriageMass = 4.0; // kg
    // The simulated encoder will return
    public static final double elevatorEncoderDistPerPulse = 2.0 * Math.PI * elevatorDrumRadius / 4096;
    private static double currentPos = 0;
    private static double currentVel = 0;

    public SimElevator()
    {
        elevatorSimEncoder = new SimEncoder("elevator");
        elevatorSim = new ElevatorSim(
        elevatorGearbox,
        elevatorGearRatio,
        elevatorCarriageMass,
        elevatorDrumRadius,
        Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT,
        Constants.ElevatorConstants.MAX_ELEVATOR_HEIGHT,
        true,
        VecBuilder.fill(0.001)
      );
      Mechanism2d mech = new Mechanism2d(3, 2);
      MechanismRoot2d root = mech.getRoot("root", 2, 0);
      elevatorMechanism = root.append(new MechanismLigament2d("elevator", Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT, 50));
      SmartDashboard.putData("Mech2d", mech);
    
    }
    @Override
    public double getEncoderSpeed() {
        return elevatorSimEncoder.getSpeed();
    }

    @Override
    public void setSpeed(double speed) {
        elevatorPower = speed;
    }

    @Override
    public double getEncoderPosition() {
        return elevatorSimEncoder.getDistance();
    }

    public void updateForSim() {
        currentPos = elevatorSimEncoder.getDistance();
        currentVel = elevatorSimEncoder.getSpeed();
        SmartDashboard.putNumber("elevator position", currentPos); 
        SmartDashboard.putNumber("elevator velocity", currentVel); 

        // sets input for elevator motor in simulation
        elevatorSim.setInput(elevatorPower * RobotController.getBatteryVoltage());
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
}



    

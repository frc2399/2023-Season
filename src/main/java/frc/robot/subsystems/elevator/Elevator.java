package frc.robot.subsystems.elevator;

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
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.SimEncoder;

public class Elevator extends SubsystemBase {

  private ElevatorIO elevatorIO;

  public static CANSparkMax elevatorMotorControllerRight;
  public static CANSparkMax elevatorMotorControllerLeft;
  public static RelativeEncoder elevatorEncoderRight;
  public static RelativeEncoder elevatorEncoderLeft;
  public static SimEncoder elevatorSimEncoder;
  public static ElevatorSim elevatorSim;

  private static double currentPos = 0;
  private static double currentVel = 0;

  public Elevator(ElevatorIO io) {
    elevatorIO = io;
  }

  @Override
  public void simulationPeriodic() {

    currentPos = elevatorSimEncoder.getDistance();
    currentVel = elevatorSimEncoder.getSpeed();
    SmartDashboard.putNumber("elevator position", currentPos); 
    SmartDashboard.putNumber("elevator velocity", currentVel); 

    // sets input for elevator motor in simulation
    elevatorSim.setInput(elevatorMotorControllerRight.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    elevatorSimEncoder.setDistance(elevatorSim.getPositionMeters());
    // sets our simulated encoder speeds
    elevatorSimEncoder.setSpeed(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    RobotContainer.elevatorMechanism.setLength(Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT + currentPos);

  }

  //returns height the elevator is at
  public double getEncoderPosition() {
    return elevatorIO.getEncoderPosition();
  }

  //returns speed of elevator
  public double getEncoderSpeed() {
    return elevatorIO.getEncoderSpeed();
  }

  public void setSpeed(double speed) {
    elevatorIO.setSpeed(speed);
  }
  
}

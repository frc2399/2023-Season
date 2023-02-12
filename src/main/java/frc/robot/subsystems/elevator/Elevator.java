package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SimEncoder;

public class Elevator extends SubsystemBase {

  private ElevatorIO elevatorIO;

  public static CANSparkMax elevatorMotorControllerRight;
  public static CANSparkMax elevatorMotorControllerLeft;
  public static RelativeEncoder elevatorEncoderRight;
  public static RelativeEncoder elevatorEncoderLeft;
  public static SimEncoder elevatorSimEncoder;
  public static ElevatorSim elevatorSim;

  public Elevator(ElevatorIO io) {
    elevatorIO = io;
  }

  @Override
  public void periodic() {
    elevatorIO.updateForSim();
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

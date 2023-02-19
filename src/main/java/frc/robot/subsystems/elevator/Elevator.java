package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

  private ElevatorIO elevatorIO;
  
  public Elevator(ElevatorIO io) {
    elevatorIO = io;
  }

  @Override
  public void periodic() {
    elevatorIO.updateForSim();
    double currentPos = getEncoderPosition();
    double currentVel = getEncoderSpeed();
    SmartDashboard.putNumber("elevator position", currentPos); 
    SmartDashboard.putNumber("elevator velocity", currentVel); 
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

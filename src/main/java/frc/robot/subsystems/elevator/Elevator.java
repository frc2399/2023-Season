package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.PIDUtil;

public class Elevator extends ProfiledPIDSubsystem {

  private ElevatorIO elevatorIO;

  // tuned values:
  private static final double feedForward = 1.666666666666667;
  private static final double kpPos = 6;

  // Trapezoidal profile constants and variables
  private static final double max_vel = 1.0;  // m/s
  private static final double max_accel = 1.0;  // m/s/s
  private static final Constraints constraints = new Constraints(max_vel, max_accel);
  private static double gravityCompensation = 0.075;

  public Elevator(ElevatorIO io) {
    super(new ProfiledPIDController(kpPos, 0, 0, constraints));
    elevatorIO = io;
  }

  @Override
  public void periodic() {
    super.periodic();
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

  @Override
  protected void useOutput(double output, State setpoint) {
    SmartDashboard.putNumber("elevator setpoint pos", setpoint.position);
    SmartDashboard.putNumber("elevator setpoint vel", setpoint.velocity);

    // Calculate the feedforward from the setpoint
    double speed = feedForward * setpoint.velocity;
    //accounts for gravity in speed
    speed += gravityCompensation; 
    // Add PID output to speed to account for error in elevator
    speed += output;
    elevatorIO.setSpeed(speed);
    
  }

  @Override
  protected double getMeasurement() {
    return elevatorIO.getEncoderPosition();
  }

  public double getGoal() {
    return m_controller.getGoal().position;
  }

    // Checks to see if elevators are within range of the setpoints
    public boolean atGoal() {
      return (PIDUtil.checkWithinRange(getGoal(), getMeasurement(), ElevatorConstants.HEIGHT_TOLERANCE));
    }
  
}

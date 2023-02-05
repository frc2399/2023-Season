package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorPositionCmd extends CommandBase {
    private final Elevator elevator;
    private final double height;
    private static DoublePublisher elevatorSetpointPublisher;
    private static DoublePublisher elevatorTargetPosPublisher;
    private static DoublePublisher elevatorTargetVelPublisher;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("datatable");

   private static final double HEIGHT_TOLERANCE = 0.01;

    // tuned values:
    private static final double feedForward = 1.666666666666667;
    private static final double kpPos = 6;
    private static final double kpVel = 0;
  
    private static double gravityCompensation = 0.075;
  
    // Trapezoidal profile constants and variables
    private static final double max_vel = 1.0;  // m/s
    private static final double max_accel = 1.0;  // m/s/s
  
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(max_vel, max_accel);
    TrapezoidProfile.State currentSetpoint;

    public SetElevatorPositionCmd (Elevator elevator, double height) {
        elevatorSetpointPublisher = datatable.getDoubleTopic("elevator setpoint").publish();
        elevatorTargetPosPublisher = datatable.getDoubleTopic("Target Pos").publish();
        elevatorTargetVelPublisher = datatable.getDoubleTopic("Target Vel").publish();
        this.elevator = elevator;
        this.height = height;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
      SmartDashboard.putNumber("Elevator Setpoint", height);
      elevatorSetpointPublisher.set(height);
      currentSetpoint = new TrapezoidProfile.State(elevator.getEncoderPosition(), 0.0);
    }

    @Override
    public void execute() {
        double currentPos = elevator.getEncoderPosition(); 
        double currentVel = elevator.getEncoderSpeed();

        // Update the profile each timestep to get the current target position and velocity
        TrapezoidProfile.State goal = new TrapezoidProfile.State(height, 0.0);
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, currentSetpoint);
        currentSetpoint = profile.calculate(0.02); // Assumes 50Hz loop. Could measure this directly

        double targetPos = currentSetpoint.position;
        double targetVel = currentSetpoint.velocity;

        elevatorTargetPosPublisher.set(targetPos);
        elevatorTargetVelPublisher.set(targetVel);

        double speed = feedForward * targetVel + kpPos * (targetPos - currentPos) + kpVel * (targetVel - currentVel);
        speed = speed + gravityCompensation;   
    
        this.elevator.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.elevator.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        double currentPos = elevator.getEncoderPosition(); 
        if (Math.abs(height-currentPos) < HEIGHT_TOLERANCE) {
            return true;
        }
       return false;
    }
}


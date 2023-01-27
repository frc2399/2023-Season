package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


public class SetElevatorPosition extends CommandBase {
    private final Elevator elevator;
    private final double height;
    private static DoublePublisher elevatorSetpointPublisher;
    private static DoublePublisher elevatorTargetPosPublisher;
    private static DoublePublisher elevatorTargetVelPublisher;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("datatable");

     // publish to the topic in "datatable" called "Out"
   DoublePublisher elevatorPositionPublisher = datatable.getDoubleTopic("elevator Pos").publish();
   DoublePublisher elevatorVelocityPublisher = datatable.getDoubleTopic("elevator Vel").publish();
   

   private static EncoderSim elevatorEncoderSim;
   private static ElevatorSim elevatorSim;

   private static final double HEIGHT_TOLERANCE = 0.2;

    // tuned values:
    private static final double feedForward = 0.55;
    private static final double kpPos = 2;
    private static final double kpVel = .2;
  
    private static double current_pos = 0;
    private static double current_vel = 0;
    private static double gravityCompensation = .075;
  
    // Trapezoidal profile constants and variables
    private static final double max_vel = 1.0;  // m/s
    private static final double max_accel = 1.0;  // m/s/s
  
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(max_vel, max_accel);
    TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State(0.0, 0.0);

    public SetElevatorPosition (Elevator elevator, double height) {
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
  
        //this code is instantiating the simulator stuff for climber
    
    }

    @Override
    public void execute() {
    EncoderSim elevatorEncoderSim = Elevator.elevatorEncoderSim;
    ElevatorSim elevatorSim = Elevator.elevatorSim;
    elevatorPositionPublisher.set( elevatorEncoderSim.getDistance());
    elevatorVelocityPublisher.set( elevatorEncoderSim.getRate());
    current_pos = elevatorEncoderSim.getDistance();
    current_vel = elevatorEncoderSim.getRate();
    //sets input for elevator motor in simulation
    elevatorSim.setInput(Elevator.motorController.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
    // elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
    //sets our simulated encoder speeds
    elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // Update the profile each timestep to get the current target position and velocity
    TrapezoidProfile.State referenceSetpoint = new TrapezoidProfile.State(height, 0.0);
    TrapezoidProfile profile = new TrapezoidProfile(constraints, referenceSetpoint, previousProfiledReference);
    previousProfiledReference = profile.calculate(0.02); // Assumes 50Hz loop. Could measure this directly

    double target_pos = previousProfiledReference.position;
    double target_vel = previousProfiledReference.velocity;

    elevatorTargetPosPublisher.set(target_pos);
    elevatorTargetVelPublisher.set(target_vel);

    double speed = feedForward * target_vel + kpPos * (target_pos - current_pos) + kpVel * (target_vel - current_vel);
    speed = speed + gravityCompensation;   
  
        this.elevator.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.elevator.setSpeed(0);
    
    }

    @Override
    public boolean isFinished() {
        current_pos = Elevator.elevatorEncoderSim.getDistance();
        if (Math.abs(height-current_pos) < HEIGHT_TOLERANCE) {
            return true;
        }
       return false;
    }
}


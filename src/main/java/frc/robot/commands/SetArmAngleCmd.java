package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmAngleCmd extends CommandBase {
    private final Arm arm;
    private final double angle;
    private static DoublePublisher armSetpointPublisher;
    private static DoublePublisher armTargetPosPublisher;
    private static DoublePublisher armTargetVelPublisher;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("datatable");

   private static final double ANGLE_TOLERANCE = 0.01;

    // tuned values:
    private static final double feedForward = 1.666666666666667;
    private static final double kpPos = 6;
    private static final double kpVel = 0;
  
    //private static double gravityCompensation = .075;
    private static double gravityCompensation = 0;
  
    // Trapezoidal profile constants and variables
    private static final double max_vel = 1.0;  // m/s
    private static final double max_accel = 1.0;  // m/s/s
  
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(max_vel, max_accel);
    TrapezoidProfile.State currentSetpoint;

    public SetArmAngleCmd (Arm arm, double angle) {
        armSetpointPublisher = datatable.getDoubleTopic("arm setpoint").publish();
        armTargetPosPublisher = datatable.getDoubleTopic("Target Pos").publish();
        armTargetVelPublisher = datatable.getDoubleTopic("Target Vel").publish();
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
      SmartDashboard.putNumber("arm Setpoint", angle);
      armSetpointPublisher.set(angle);
      currentSetpoint = new TrapezoidProfile.State(arm.getEncoderPosition(), 0.0);
    }

    @Override
    public void execute() {
        double current_pos = arm.getEncoderPosition(); //radians
        double current_vel = arm.getEncoderSpeed();

        // Update the profile each timestep to get the current target position and velocity
        TrapezoidProfile.State goal = new TrapezoidProfile.State(angle, 0.0);
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, currentSetpoint);
        currentSetpoint = profile.calculate(0.02); // Assumes 50Hz loop. Could measure this directly

        double target_pos = currentSetpoint.position;
        double target_vel = currentSetpoint.velocity;

        armTargetPosPublisher.set(target_pos);
        armTargetVelPublisher.set(target_vel);

        double speed = feedForward * target_vel + kpPos * (target_pos - current_pos) + kpVel * (target_vel - current_vel);
        speed = speed + gravityCompensation;   
    
        this.arm.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.arm.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        double current_pos = arm.getEncoderPosition(); 
        if (Math.abs(angle-current_pos) < ANGLE_TOLERANCE) {
            return true;
        }
       return false;
    }
}


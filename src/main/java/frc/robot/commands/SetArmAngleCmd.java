package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class SetArmAngleCmd extends CommandBase {
    private final Arm arm;
    private double lastTargetAngle; 

    private static final double ANGLE_TOLERANCE = 0.1;

    // tuned values:
    private static final double feedForward = 0.1/6 * 1.17;
    private static final double kpPos = 0.8;
    private static final double kpVel = 0;
  
    private static double gravityCompensation = 0.11;
  
    // Trapezoidal profile constants and variables
    private static final double max_vel = 1.0;  // rad/s
    private static final double max_accel = 1.8;  // rad/s/s
  
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(max_vel, max_accel);
    TrapezoidProfile.State currentSetpoint;

    public SetArmAngleCmd (Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        currentSetpoint = new TrapezoidProfile.State(arm.getEncoderPosition(), arm.getEncoderSpeed());
      
    }

    @Override
    public void execute() {
        double targetAngle = arm.getTargetAngle();
        //update the trapezoid profile if the goal angle is changed so new profile starts at current arm position
        if(targetAngle != lastTargetAngle) {
            currentSetpoint = new TrapezoidProfile.State(arm.getEncoderPosition(), arm.getEncoderSpeed());
        }
        lastTargetAngle = targetAngle;
        SmartDashboard.putNumber("arm Setpoint", targetAngle);
        
        double current_pos = arm.getEncoderPosition(); //radians
        double current_vel = arm.getEncoderSpeed();

        // Update the profile each timestep to get the current target position and velocity
        TrapezoidProfile.State goal = new TrapezoidProfile.State(targetAngle, 0.0);
        //setpoint is where ur going next, goal is where u end up at the very end
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, currentSetpoint); 
        currentSetpoint = profile.calculate(0.02); // Assumes 50Hz loop. Could measure this directly

        double target_pos = currentSetpoint.position;
        double target_vel = currentSetpoint.velocity;

        SmartDashboard.putNumber("arm target pos", target_pos);
        SmartDashboard.putNumber("arm target vel", target_vel);

        double speed =  feedForward * target_vel + kpPos * (target_pos - current_pos) + kpVel * (target_vel - current_vel);
        speed += gravityCompensation * Math.cos(current_pos);  
    
        this.arm.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.arm.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        //never ends
        return false;
    }
}


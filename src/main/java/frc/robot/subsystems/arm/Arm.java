// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.PIDUtil;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.Commands;
 import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
 
//  /** A robot arm subsystem that moves with a motion profile. */
//  public class ArmSubsystem extends TrapezoidProfileSubsystem {
//    private final ExampleSmartMotorController m_motor =
//        new ExampleSmartMotorController(ArmConstants.kMotorPort);
   
 
//    /** Create a new ArmSubsystem. */
//    public ArmSubsystem() {
     
//      m_motor.setPID(ArmConstants.kP, 0, 0);
//    }
 
   
 
//    public Command setArmGoalCommand(double kArmOffsetRads) {
//      return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
//    }
//  }

public class Arm extends TrapezoidProfileSubsystem {
  /** Creates a new Arm. */
  private ArmIO armIO;
  private double targetAngle = - Math.PI/2; 
  //private static final double feedForward = 0.133;
  private static final double feedForward = 0.14285;

  private static final double kpPos = 0.8;

  // Trapezoidal profile constants and variables
  private static final double max_vel = 9.0;  // rad/s (NEO specs / gear ratio, converted into rad/s)
  private static final double max_accel = 8;  // rad/s/s (2.7)
  private static final Constraints constraints = new Constraints(max_vel, max_accel);
  private static double gravityCompensation = 0.04;

  //only using kg for now; can tune ks later.
  private final ArmFeedforward m_feedforward =
       new ArmFeedforward(
           0.0, .04,
           1 / max_vel, 0);

  public Arm(ArmIO io) {
    super(
         new TrapezoidProfile.Constraints(
             max_vel, max_accel),
         ArmConstants.INITIAL_OFFSET);
    //super(new ProfiledPIDController(kpPos, 0, 0, constraints));
    armIO = io;
  }

  @Override
  public void periodic() {
    // Call periodic method in profile pid subsystem to prevent overriding
    super.periodic();
    armIO.periodicUpdate();

    SmartDashboard.putNumber("arm/goal position", getGoal());
    SmartDashboard.putNumber("arm/velocity", getEncoderSpeed()); 
    SmartDashboard.putNumber("arm/position", getEncoderPosition()); 
    RobotContainer.armMechanism.setAngle(Units.radiansToDegrees(getEncoderPosition()) - 50);
    SmartDashboard.putNumber("arm/motor speed(pct)", armIO.getMotorDutyCycle());
  
  }
  
  public  double getEncoderPosition() {
    return armIO.getEncoderPosition();
  }

  public double getEncoderSpeed() {
    return armIO.getEncoderSpeed();
  }

  public void setSpeed(double speed) {
    speed = Math.max(Math.min(speed, 0.5), -0.5);
    armIO.setSpeed(speed);
    SmartDashboard.putNumber("arm/speed", speed);
  }

  public double getTargetAngle() {
    return targetAngle; 
  }

  public void setTargetAngle(double angle) {
    DataLogManager.log("Set target to " + angle);
    targetAngle = angle; 
  }

  public void setSpeedGravityCompensation(double speed) {
    // calls set speed function in the file that does armIO.setSpeed after capping speed
    setSpeed(speed + gravityCompensation * Math.cos(getEncoderPosition()));
  }

  public double getArmCurrent() {
    return armIO.getArmCurrent();
  }

  // @Override
  // protected void useOutput(double output, State setpoint) {
  //   SmartDashboard.putNumber("arm/setpoint pos", setpoint.position);
  //   SmartDashboard.putNumber("arm/setpoint vel", setpoint.velocity);

  //   // Calculate the feedforward from the setpoint
  //   double speed = feedForward * setpoint.velocity;
  //   //accounts for gravity in speed
  //   speed += gravityCompensation * Math.cos(getEncoderPosition()); 
  //   // Add PID output to speed to account for error in arm
  //   speed += output;
  //   // calls set speed function in the file that does armIO.setSpeed after capping speed
  //   setSpeed(speed);
  // }

  @Override
   public void useState(TrapezoidProfile.State setpoint) {
     // Calculate the feedforward from the sepoint
     double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
     // Add the feedforward to the PID output to get the motor output
     armIO.setSetpoint(setpoint, feedforward);

     SmartDashboard.putNumber("arm/setpoint position", setpoint.position);
     SmartDashboard.putNumber("arm/setpoint velocity", setpoint.velocity);
   }

  
  public double getMeasurement() {
    return armIO.getEncoderPosition();
  }

  public double getGoal() {
     return targetAngle;
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    targetAngle = kArmOffsetRads;
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

    // Checks to see if arm is within range of the setpoints
    public boolean atGoal() {
      return (PIDUtil.checkWithinRange(getGoal(), getMeasurement(), ArmConstants.ANGLE_TOLERANCE_AUTON));
    }
  
  public void setPosition(double position) {
    armIO.setPosition(position);
  }
}

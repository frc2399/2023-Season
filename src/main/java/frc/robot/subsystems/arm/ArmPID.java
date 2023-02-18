package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmPID extends ProfiledPIDSubsystem implements ArmIO {

    private ArmIO armIO;
    private static CANSparkMax armMotorController;
    public static RelativeEncoder armEncoder;
    ArmFeedforward armFeedforward;

    // TODO: find actual values (took from setArmAngleCmd)
    private static final double kpPos = 0.8;
    private static final double kpVel = 0;
    private static final double kSVolts = 0.1/6 * 1.17;
    private static final double kGVolts = 0.11 ;
    private static final double kP = 0;

    private static final double kArmOffsetRads = 0;

    // Trapezoidal profile constants and variables
    private static final double max_vel = 1.0;  // rad/s
    private static final double max_accel = 1.8;  // rad/s/s

  /** Create a new ArmSubsystem. */
  public ArmPID() {

    super(
        new ProfiledPIDController(
            kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                max_vel,
                max_accel)),
        0);
        
    // Start arm at rest in neutral position
    setGoal(kArmOffsetRads);

    armMotorController = new CANSparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotorController.restoreFactoryDefaults();
    armEncoder = armMotorController.getEncoder();
    armMotorController.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armMotorController.setInverted(true);
    armEncoder.setPosition(0);

    armEncoder.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
    armFeedforward =
      new ArmFeedforward(
          kSVolts, kGVolts,
          kpVel);
  }

  @Override
  public double getEncoderPosition() {
      return armEncoder.getPosition();
  }

  @Override
  public double getEncoderSpeed() {
      return armEncoder.getVelocity();
  }

  @Override
  public void setSpeed(double speed) {
      armMotorController.set(speed);
      SmartDashboard.putNumber("ArmSpeed", speed);
  }

  @Override
  public void updateForSim(){
      
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    armMotorController.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return armEncoder.getPosition() + kArmOffsetRads;
  }
}
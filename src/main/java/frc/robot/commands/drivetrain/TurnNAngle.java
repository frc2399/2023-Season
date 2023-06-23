package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DanceConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.RealDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Rotates the robot by an angle (value passed through the constructor) 
 * Calculates the angle that the robot should reach by adding its current angle to the angle passed through the constructor
 * Used for autonomous commands
 */

public class TurnNAngle extends CommandBase {
  /** Creates a new TurnToNAngle. */
  public double turnAngle;
  private final DriveTrain m_driveTrain;
  private double currentAngle;
  double newAngle;
  double speed;

  public TurnNAngle(double speed, double turnAngle, DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turnAngle = turnAngle;
    this.speed = speed;
    m_driveTrain = subsystem;
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurnNAngle initialized, turnAngle: " + turnAngle);


    currentAngle = Units.degreesToRadians(RealDrive.ahrs.getAngle());
    
    currentAngle = modAngle(currentAngle);
    
    newAngle = modAngle(currentAngle + turnAngle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   
    currentAngle = Units.degreesToRadians(RealDrive.ahrs.getAngle());

    currentAngle = modAngle(currentAngle);

    double error = newAngle - currentAngle;
    System.out.println("error angle " +  error);

    error = modAngle(error);

    double speed;
    
    speed = DanceConstants.kP * error;

    speed = MathUtil.clamp(speed, -DanceConstants.DANCE_SLOW_SPEED_FRACTION, DanceConstants.DANCE_SLOW_SPEED_FRACTION);

    m_driveTrain.setMotors(speed, -speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_driveTrain.setMotors(0, 0);
    
    System.out.println("TurnNangle ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double errorTolerance = DanceConstants.ANGLE_ERROR_TOLERANCE;
    //System.out.println("difference " + Math.abs(modAngle(newAngle - currentAngle)));
    if (Math.abs(modAngle(newAngle - currentAngle)) <= errorTolerance) {
      return true;
    }
    return false;
  }

  public double modAngle(double value) {
    return ((value + Math.PI) % (2 * Math.PI)) - Math.PI;
  }
}

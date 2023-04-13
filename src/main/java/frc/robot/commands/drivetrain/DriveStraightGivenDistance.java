package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;

/**
 * Drives the robot forward a given distance (relative to the robot's position upon initialization), and a given speed
 * only used for shuffleboard testing, not used in any commands
 **/

public class DriveStraightGivenDistance extends CommandBase {

    //insantiate global variables
    double currentPosition;
    double targetDistanceMeters;
    double newTargetDistance;
    double startAngle, currentAngle;
    double speedLimit;
    DriveTrain m_driveTrain;
    private SlewRateLimiter driveLimiter;

    
 
	public DriveStraightGivenDistance(double targetDistanceMeters, double speedLimit, DriveTrain subsystem) {
        
        //initialize variables
        this.targetDistanceMeters = targetDistanceMeters;
        this.speedLimit = speedLimit;
        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

    }
    

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
        // Sets the current position to where robot is starting
        currentPosition = (
            m_driveTrain.getLeftEncoderMeters() + 
            m_driveTrain.getRightEncoderMeters() )/ 2;
        System.out.println("starting current position " + currentPosition);
        DataLogManager.log("DriveStraightGivenDistance started");

        startAngle = m_driveTrain.getPoseMeters().getRotation().getRadians();

        // find distance robot needs to travel to from its current position
        newTargetDistance = currentPosition + targetDistanceMeters;

        //slew rate limiter to remove aggression :(
        this.driveLimiter = new SlewRateLimiter(0.75);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        // Get the average position between leftEncoder and rightEncoder
        currentPosition = (m_driveTrain.getLeftEncoderMeters() + m_driveTrain.getRightEncoderMeters()) / 2;
        currentAngle = m_driveTrain.getPoseMeters().getRotation().getRadians();

        double distanceError = newTargetDistance - currentPosition;
        double angleError = startAngle - currentAngle;

        double outputSpeed = (1 * distanceError);
        outputSpeed = MathUtil.clamp(outputSpeed, -0.3, 0.3);
        outputSpeed  = driveLimiter.calculate(outputSpeed);

        outputSpeed *= speedLimit;

        double straightCorrection = 1.0 * modAngle(angleError);
        straightCorrection = MathUtil.clamp(straightCorrection, -0.5 * Math.abs(outputSpeed), 0.5 * Math.abs(outputSpeed));

        m_driveTrain.setMotors(outputSpeed - straightCorrection, outputSpeed + straightCorrection);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // increased error tolerance so the command will finish in auton
        double butteryErrorTolerance = 0.05;
        // SmartDashboard.getNumber("Error Tolerance Distance", 0.5);
        // SmartDashboard.putNumber("distance bt td and cp", Math.abs(targetDistance - currentPosition));
        // System.out.println("distance bt td and cp " +  Math.abs(td - cp));

        if (Math.abs(newTargetDistance - currentPosition) <= butteryErrorTolerance)
        {
            return true;
        }
        return false;
        
    }

    public double modAngle(double value) {
        value = (value + Math.PI) % (Math.PI * 2);  // Take "remainder" (https://stackoverflow.com/a/2172061)
        value = value < 0 ? value + Math.PI * 2 : value;  // If less then 0, add the value to make it "modulus"
        return value - Math.PI;  // Subtract PI to make the angle in the range -PI to PI
      }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    
        m_driveTrain.setMotors(0, 0);

        DataLogManager.log("DriveStraightGivenDistance ended");

    }
}
package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;

import java.util.function.Supplier;

public class ArcadeDriveCmd extends CommandBase {

    private final DriveTrain driveSubsystem;
    private SlewRateLimiter driveLimiter;
    private SlewRateLimiter turnLimiter;
    public static boolean isSlow = false;
    private final Supplier<Double> speedFunction, turnFunction;

    /* This command does this (fill in)... */
    public ArcadeDriveCmd(DriveTrain driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        double driveSlew = SmartDashboard.getNumber("drive slew", XboxConstants.DRIVE_SLEW_RATE);
        this.driveLimiter = new SlewRateLimiter(5.0);
        this.turnLimiter = new SlewRateLimiter(5.0);
        this.driveSubsystem = driveSubsystem;
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        addRequirements(driveSubsystem);
    }
    

    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
        isSlow = true;
    }



    @Override
    public void execute() {
        double realTimeSpeed;
        double realTimeTurn;

        // calculate real time speed
        //inverting to make forwards positive
        realTimeSpeed = -speedFunction.get();
        SmartDashboard.putNumber("Speed function", realTimeSpeed);
        if (Math.abs(realTimeSpeed) <= XboxConstants.FORWARD_DEADBAND) {
            realTimeSpeed = 0;
        } 
        realTimeSpeed  = driveLimiter.calculate(realTimeSpeed);
        SmartDashboard.putNumber("Real Time Speed", realTimeSpeed);

        //calculate real time turn
        //inverting to make left positive (ccw)
        realTimeTurn = -turnFunction.get();
        SmartDashboard.putNumber("Turn function", realTimeTurn);

        if (Math.abs(realTimeTurn) <= XboxConstants.TURN_DEADBAND) {
            realTimeTurn = 0.0;
        }

        double a = DriveConstants.TURN_SENSITIVITY;
        realTimeTurn = ((1 - a) * realTimeTurn) + (a * Math.pow(realTimeTurn, 3));
        
        realTimeTurn = turnLimiter.calculate(realTimeTurn);
        SmartDashboard.putNumber("Real Time Turn", realTimeTurn);

        double left = realTimeSpeed - realTimeTurn;
        SmartDashboard.putNumber("Left Speed", left);
        double right = realTimeSpeed + realTimeTurn;
        SmartDashboard.putNumber("Right Speed", right);


        if (isSlow) {        
            this.driveSubsystem.setMotors(left * DriveConstants.SLOW_SPEED_FRACTION, right * DriveConstants.SLOW_SPEED_FRACTION);
        }
        else {
            this.driveSubsystem.setMotors(left, right);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
        isSlow = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
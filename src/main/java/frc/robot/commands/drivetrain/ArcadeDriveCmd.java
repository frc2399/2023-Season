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
    public static boolean isSlow = false;
    private final Supplier<Double> speedFunction, turnFunction;

    /* This command does this (fill in)... */
    public ArcadeDriveCmd(DriveTrain driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        double driveSlew = SmartDashboard.getNumber("drive slew", XboxConstants.DRIVE_SLEW_RATE);
        this.driveLimiter = new SlewRateLimiter(driveSlew);
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
        double val;

        // calculate real time speed
        val = speedFunction.get();
        if (Math.abs(val) <= XboxConstants.FORWARD_DEADBAND) {
            val = 0;
        } 
        realTimeSpeed  = driveLimiter.calculate(val);

        //calculate real time turn
        val = turnFunction.get();
        if (Math.abs(val) <= XboxConstants.TURN_DEADBAND) {
            val = 0.0;
        }
        // inverting :)
        val = -val;
        double a = DriveConstants.TURN_SENSITIVITY;
        val = ((1 - a) * val) + (a * Math.pow(val, 3));
        realTimeTurn = driveLimiter.calculate(val);
        
        double left = realTimeSpeed - realTimeTurn;
        double right = realTimeSpeed + realTimeTurn;

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
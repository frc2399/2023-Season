package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.util.DriveUtil;

import java.util.function.Supplier;

public class ArcadeDriveCmd extends CommandBase {

    private final DriveTrain driveSubsystem;
    private SlewRateLimiter driveLimiter;
    private SlewRateLimiter turnLimiter;
    public static boolean isSlow = true;
    private final Supplier<Double> speedFunction, turnFunction;

    /* This command does this (fill in)... */
    public ArcadeDriveCmd(DriveTrain driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.driveLimiter = new SlewRateLimiter(5.0);
        this.turnLimiter = new SlewRateLimiter(5.0);
        this.driveSubsystem = driveSubsystem;
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        addRequirements(driveSubsystem);
    }
    

    @Override
    public void initialize() {
        DataLogManager.log("ArcadeDriveCmd started!");
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
        realTimeSpeed = DriveUtil.computeDeadband(realTimeSpeed, XboxConstants.FORWARD_DEADBAND);
        realTimeSpeed  = driveLimiter.calculate(realTimeSpeed);
        SmartDashboard.putNumber("Real Time Speed", realTimeSpeed);

        //calculate real time turn
        //inverting to make left positive (ccw)
        realTimeTurn = -turnFunction.get();
        SmartDashboard.putNumber("Turn function", realTimeTurn);
    
        realTimeTurn = DriveUtil.computeDeadband(realTimeTurn, XboxConstants.TURN_DEADBAND);

        
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
        DataLogManager.log("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
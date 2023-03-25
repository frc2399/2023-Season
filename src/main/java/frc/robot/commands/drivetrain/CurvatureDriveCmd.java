package frc.robot.commands.drivetrain;


import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.util.DriveUtil;


public class CurvatureDriveCmd extends CommandBase {


    private final DriveTrain driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction, elevatorHeight;
    Debouncer m_debouncer;
    private SlewRateLimiter driveLimiter;

    private final double percentTurningSpeed = 0.6;
    private final double driveDeadband = 0.05;
    private final double turnDeadband = 0.05;

    public static boolean isSlow = false;


    /* This command does this (fill in)... */
    public CurvatureDriveCmd(DriveTrain driveSubsystem,
            Supplier<Double> speedFunction, Supplier<Double> turnFunction, Supplier<Double> elevatorHeight) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        this.elevatorHeight = elevatorHeight;
        addRequirements(driveSubsystem);
        this.driveLimiter = new SlewRateLimiter(4.0);
        // Creates a Debouncer in "both" mode.
        m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);


        // So if currently false the signal must go true for at least .1 seconds before being read as a True signal.


    }


    @Override
    public void initialize() {
        //System.out.println("CurvatureDriveCmd started!");
        isSlow = false;
    }


    @Override
    public void execute() {

        double realTimeSpeed;
        double realTimeTurn;
        double right;
        double left;


        realTimeSpeed = -speedFunction.get();
        realTimeTurn = -turnFunction.get();
    
        //transforming the drive value
        SmartDashboard.putNumber("drive/joystick for/back (%)", realTimeSpeed);
        SmartDashboard.putNumber("drive/joystick turn (%)", realTimeTurn);

        realTimeSpeed = realTimeSpeed * realTimeSpeed * Math.signum(realTimeSpeed);
        //realTimeSpeed = realTimeSpeed * realTimeSpeed * realTimeSpeed;
        //realTimeSpeed = 0.9 * realTimeSpeed * realTimeSpeed * realTimeSpeed + 0.1 * Math.signum(realTimeSpeed);
        realTimeSpeed  = driveLimiter.calculate(realTimeSpeed);
        realTimeSpeed = DriveUtil.computeDeadband(realTimeSpeed, driveDeadband);
        // SmartDashboard.putNumber("Transformed Joystick Value", realTimeSpeed);


        //transforming the turn value
        // SmartDashboard.putNumber("Raw Turn Value", realTimeTurn);
        realTimeTurn = DriveUtil.computeDeadband(realTimeTurn, turnDeadband);
        realTimeTurn = realTimeTurn * realTimeTurn * Math.signum(realTimeTurn);
        // SmartDashboard.putNumber("Transformed Turn Value", realTimeTurn);

        if(m_debouncer.calculate(Math.abs(realTimeSpeed) <= Constants.XboxConstants.FORWARD_DEADBAND) && realTimeTurn != 0)
        {
            // Do something now that the DI is True.
            left = realTimeSpeed - realTimeTurn * percentTurningSpeed;
            right = realTimeSpeed + realTimeTurn * percentTurningSpeed;
        }
        else
        {
            //Multiplied by realTimeSpeed to make turn speed proportional to straight speed
            //Speed and turn proportional so arc remains the same when the speed changes
            left = realTimeSpeed - realTimeTurn * Math.abs(realTimeSpeed);
            right = realTimeSpeed + realTimeTurn * Math.abs(realTimeSpeed);
            //System.out.println("Curvature drive!");
        }


        //Makes it so the ratio between speed and turn are still the same if turn * speed is >1
        double maxValue = Math.max(Math.max(Math.abs(left), Math.abs(right)), 1);
        left /= maxValue;
        right /= maxValue;

        if (isSlow || elevatorHeight.get() > ElevatorConstants.MAX_ELEVATOR_HEIGHT / 2) {        
            this.driveSubsystem.setMotors(left * DriveConstants.SLOW_SPEED_FRACTION, right * DriveConstants.SLOW_SPEED_FRACTION);
            SmartDashboard.putBoolean("drive/slow mode", true);
        }
        else {
            // SmartDashboard.putNumber("left motor speed", left);
            // SmartDashboard.putNumber("right motor speed", right);
            this.driveSubsystem.setMotors(left, right);
            SmartDashboard.putBoolean("drive/slow mode", false);
        }
        
    }


    @Override
    public void end(boolean interrupted) {
        //System.out.println("CurvatureDriveCmd ended!");
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}

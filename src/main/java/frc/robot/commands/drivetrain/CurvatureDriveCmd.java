package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import java.util.function.Supplier;

public class CurvatureDriveCmd extends CommandBase {

    private final DriveTrain driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;
    Timer timer;
    Debouncer m_debouncer;

    /* This command does this (fill in)... */
    public CurvatureDriveCmd(DriveTrain driveSubsystem, //
            Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        timer = new Timer();
        // Creates a Debouncer in "both" mode.
        Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

        // So if currently false the signal must go true for at least .1 seconds before being read as a True signal.

    }

    @Override
    public void initialize() {
        System.out.println("CurvatureDriveCmd started!");
        timer.reset();
    }

    @Override
    public void execute() {
        double realTimeSpeed;
        double realTimeTurn;
        double right;
        double left;

        realTimeSpeed = speedFunction.get();
        realTimeTurn = -turnFunction.get();

        //Multiplied by realTimeSpeed to make turn speed proportional to straight speed
        //Speed and turn proportional so arc remains the same when the speed changes
        // if(quickTurn.get() == true)
        // {
        //     left = realTimeSpeed - realTimeTurn;
        //     right = realTimeSpeed + realTimeTurn;
        // }
        if(Math.abs(realTimeSpeed) <= Constants.XboxConstants.FORWARD_DEADBAND && realTimeTurn != 0)
        {
            timer.start();
            System.out.println(timer.get());
            //feed the watchdog borkborkborkborkborkborkborkbork
            if (m_debouncer.calculate(Math.abs(realTimeSpeed) <= Constants.XboxConstants.FORWARD_DEADBAND)) {
                // Do something now that the DI is True.
                left = realTimeSpeed - realTimeTurn;
                right = realTimeSpeed + realTimeTurn;
            }
            else {
                left = realTimeSpeed - realTimeTurn * Math.abs(realTimeSpeed);
                right = realTimeSpeed + realTimeTurn * Math.abs(realTimeSpeed);
            }
            // while(timer.get() < 0.1)
            // {
            //     //curvature drive deadband
            //     left = realTimeSpeed - realTimeTurn * Math.abs(realTimeSpeed);
            //     right = realTimeSpeed + realTimeTurn * Math.abs(realTimeSpeed);
            //     //System.out.println("Curvature drive deadband!");
            // }
            //arcade drive to turn in place
            left = realTimeSpeed - realTimeTurn;
            right = realTimeSpeed + realTimeTurn;
            //System.out.println("arcade drive!");
        }
        else
        {
            timer.reset();
            timer.stop();
            //curvature drive
            left = realTimeSpeed - realTimeTurn * Math.abs(realTimeSpeed);
            right = realTimeSpeed + realTimeTurn * Math.abs(realTimeSpeed);
            //System.out.println("Curvature drive!");
        }

        //Makes it so the ratio between speed and turn are still the same if turn * speed is >1
        double maxValue = Math.max(Math.max(Math.abs(left), Math.abs(right)), 1);
        left /= maxValue;
        right /= maxValue;
        this.driveSubsystem.setMotors(left, right);   
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("CurvatureDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
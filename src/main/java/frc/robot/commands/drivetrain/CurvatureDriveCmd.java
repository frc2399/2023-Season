package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class CurvatureDriveCmd extends CommandBase {

    private final DriveTrain driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;

    /* This command does this (fill in)... */
    public CurvatureDriveCmd(DriveTrain driveSubsystem, //
            Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CurvatureDriveCmd started!");
    }

    @Override
    public void execute() {
        double realTimeSpeed;
        double realTimeTurn;

        realTimeSpeed = speedFunction.get();
        realTimeTurn = turnFunction.get();

        //Multiplied by realTimeSpeed to make turn speed proportional to straight speed
        //Speed and turn proportional so arc remains the same when the speed changes
        double left = realTimeSpeed - realTimeTurn * realTimeSpeed;
        double right = realTimeSpeed + realTimeTurn * realTimeSpeed;
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
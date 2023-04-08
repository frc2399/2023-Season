package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;

public class StallIntakeCmd extends CommandBase {

    private final Intake intakeSubsystem;
    private final Supplier<Boolean> intake, outtake;
    Debouncer debouncer;
    Timer timer;
    double intakeSpeed;
    int intakeCurrentLimit;
    double velocityThreshold = 100;
    String intakeState;

    public StallIntakeCmd(Intake intakeSubsystem, Supplier<Boolean> intake, Supplier<Boolean> outtake) {
        this.intakeSubsystem = intakeSubsystem;
        this.intake = intake;
        this.outtake = outtake;
        //timer in constructor so it doesn't automatically intake when robot starts
        timer = new Timer();
        timer.start();
        addRequirements(intakeSubsystem);
    }

      // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        debouncer = new Debouncer(0.15);
        intakeSpeed = 0.0;
        intakeCurrentLimit = 0;
    }

    @Override
    public void execute() {
        if (intake.get()) {
            intakeState = "intaking";
            timer.reset();
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_IN_SPEED : IntakeConstants.CUBE_IN_SPEED;
            intakeCurrentLimit = RobotContainer.coneMode ? IntakeConstants.CONE_IN_CURRENT : IntakeConstants.CUBE_IN_CURRENT;
            if (debouncer.calculate(Math.abs(intakeSubsystem.getEncoderSpeed()) < velocityThreshold)) {
                Intake.isIntooked = true;
            }
        }
        else if (outtake.get()) {
            intakeState = "outtaking";
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_OUT_SPEED : IntakeConstants.CUBE_OUT_SPEED;
            intakeCurrentLimit = IntakeConstants.OUT_CURRENT;
            Intake.isIntooked = false;
        }
        else if (Intake.isIntooked)
        {
            intakeState = "stalling";
            // need more beans to intake
            intakeSpeed = 0.5 * (RobotContainer.coneMode ? IntakeConstants.CONE_IN_SPEED : IntakeConstants.CUBE_IN_SPEED);
            intakeCurrentLimit = 3;
        }
        // increased timer 
        else if (!timer.hasElapsed(1.5)) {
            intakeState = "slurping";
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_IN_SPEED : IntakeConstants.CUBE_IN_SPEED;
            if (debouncer.calculate(Math.abs(intakeSubsystem.getEncoderSpeed()) < velocityThreshold)) {
                Intake.isIntooked = true;
            }
        }
        else {
            intakeState = "nothing";
            intakeSpeed = 0;
            intakeCurrentLimit = 3;
        }
        // reset when timer has elapsed and not intaking
        if (!intake.get() && timer.hasElapsed(1.5)) {
            debouncer.calculate(false);
        }
        intakeSubsystem.setMotor(intakeSpeed);
        intakeSubsystem.setCurrentLimit(intakeCurrentLimit);
        SmartDashboard.putString("intake/state", intakeState);
        SmartDashboard.putNumber("Intake current limit", intakeCurrentLimit);
        SmartDashboard.putBoolean("intake/isIntooked", Intake.isIntooked);
        SmartDashboard.putNumber("intake/encoder speed", intakeSubsystem.getEncoderSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("StallIntakeCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

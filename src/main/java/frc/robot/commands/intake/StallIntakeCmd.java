package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;

public class StallIntakeCmd extends CommandBase {

    private final Intake intakeSubsystem;
    private final Supplier<Boolean> intake, outtake;
    Debouncer debouncer;
    double intakeSpeed;
    int intakeCurrentLimit;
    double velocityThreshold = 100;

    public StallIntakeCmd(Intake intakeSubsystem, Supplier<Boolean> intake, Supplier<Boolean> outtake) {
        this.intakeSubsystem = intakeSubsystem;
        this.intake = intake;
        this.outtake = outtake;
        addRequirements(intakeSubsystem);
    }

      // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        debouncer = new Debouncer(0.5);
        intakeSpeed = 0.0;
        intakeCurrentLimit = 0;
    }

    @Override
    public void execute() {
        if (intake.get()) {
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_IN_SPEED : IntakeConstants.CUBE_IN_SPEED;
            intakeCurrentLimit = RobotContainer.coneMode ? IntakeConstants.CONE_IN_CURRENT : IntakeConstants.CUBE_IN_CURRENT;
            if (debouncer.calculate(Math.abs(intakeSubsystem.getEncoderSpeed()) < velocityThreshold)) {
                Intake.isIntooked = true;
                //intakeSubsystem.setCurrentLimit(3);
            }
        }
        else if (outtake.get()) {
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_OUT_SPEED : IntakeConstants.CUBE_OUT_SPEED;
            intakeCurrentLimit = IntakeConstants.OUT_CURRENT;
            Intake.isIntooked = false;
            // reset debouncer hacky way
            debouncer.calculate(false);
        }
        else if (Intake.isIntooked)
        {
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_IN_SPEED : IntakeConstants.CUBE_IN_SPEED;
            intakeCurrentLimit = 3;
        }
        else {
            intakeSpeed = 0;
            intakeCurrentLimit = 3;
        }
        
        intakeSubsystem.setMotor(intakeSpeed);
        intakeSubsystem.setCurrentLimit(intakeCurrentLimit);
        SmartDashboard.putNumber("Intake current limit", intakeCurrentLimit);
        SmartDashboard.putBoolean("isIntooked", Intake.isIntooked);
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

package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;

public class StallIntakeCmd extends CommandBase {

    private final Intake intakeSubsystem;
    private final Supplier<Boolean> intake, outtake;
    Debouncer debouncer;
    double currentPos, lastPos;
    double intakeSpeed;
    int intakeCurrent;

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
        intakeCurrent = 0;
        lastPos = intakeSubsystem.getEncoderPosition();
    }

    @Override
    public void execute() {

        if (intake.get()) {
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_IN_SPEED : IntakeConstants.CUBE_IN_SPEED;
            intakeCurrent = RobotContainer.coneMode ? IntakeConstants.CONE_IN_CURRENT : IntakeConstants.CUBE_IN_CURRENT;
        }
        if (outtake.get()) {
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_OUT_SPEED : IntakeConstants.CUBE_OUT_SPEED;
            intakeCurrent = IntakeConstants.OUT_CURRENT;
        }
        currentPos = intakeSubsystem.getEncoderPosition();
        
        if (debouncer.calculate(Math.abs(currentPos - lastPos) < 2)) {
            intakeSubsystem.setCurrentLimit(3);
        }
        else {
            intakeSubsystem.setMotor(intakeSpeed);
            intakeSubsystem.setCurrentLimit(intakeCurrent);
            lastPos = currentPos;
        }
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

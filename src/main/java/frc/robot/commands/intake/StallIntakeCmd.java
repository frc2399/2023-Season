package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.robot.IntakeConeFromGround;
import frc.robot.subsystems.intake.Intake;

public class StallIntakeCmd extends CommandBase {

    private final Intake intakeSubsystem;
    private final Supplier<Boolean> intake, outtake;;

    public StallIntakeCmd(Intake intakeSubsystem, Supplier<Boolean> intake, Supplier<Boolean> outtake) {
        this.intakeSubsystem = intakeSubsystem;
        this.intake = intake;
        this.outtake = outtake;
        addRequirements(intakeSubsystem);
    }

      // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double intakeSpeed = 0;
        if (intake.get()) {
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_IN_SPEED : IntakeConstants.CUBE_IN_SPEED;
        }
        if (outtake.get()) {
            intakeSpeed = RobotContainer.coneMode ? IntakeConstants.CONE_OUT_SPEED : IntakeConstants.CUBE_OUT_SPEED;
        }
        intakeSubsystem.setMotor(intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

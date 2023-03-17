// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;

public class IntakeUntilStall extends CommandBase {
  private Intake intake;
  double speed;
  SlewRateLimiter filter;
  Debouncer debouncer;
  private double velocityThreshold = 100;
  private double currentThreshold = 20;
  /** Creates a new CollectPieceCmd. */
  public IntakeUntilStall(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    debouncer = new Debouncer(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //speed = filter.calculate(targetSpeed);
    if (RobotContainer.coneMode) {
    intake.setMotor(IntakeConstants.CONE_IN_SPEED);
    }
    else {
    intake.setMotor(IntakeConstants.CUBE_IN_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intake.openRight();
    // intake.openLeft();
    System.out.println("end intakeUntilStall");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (debouncer.calculate(Math.abs(intake.getEncoderSpeed()) < velocityThreshold)) {
      Intake.isIntooked = true;
      System.out.println("finished intakeUntilStall");
      return true;
      // Stalled!
    }
    return false;
  }


}

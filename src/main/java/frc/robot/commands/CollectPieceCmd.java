// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class CollectPieceCmd extends CommandBase {
  SlewRateLimiter filter;
  private Intake intake;
  private double speed;
  private static double targetSpeed = 1.0;
  /** Creates a new CollectPieceCmd. */
  public CollectPieceCmd(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    filter = new SlewRateLimiter(IntakeConstants.INTAKE_SLEW_RATE);
    intake.closeRight();
    // intake.closeLeft();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = filter.calculate(targetSpeed);
    intake.setMotor(IntakeConstants.INTAKE_IN_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.openRight();
    // intake.openLeft();
    intake.setMotor(0);
    System.out.println("speed set to 0");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

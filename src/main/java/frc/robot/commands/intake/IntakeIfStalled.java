// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;

public class IntakeIfStalled extends CommandBase {
  private Intake intake;
  double maintainSpeedFraction = 0.05;
  /* Meeeeeeeeep */
  public IntakeIfStalled(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Meow
  @Override
  public void initialize() {}

  // Bork
  @Override
  public void execute() {
    if (RobotContainer.coneMode) {
      intake.setMotor(Constants.IntakeConstants.CONE_IN_SPEED * maintainSpeedFraction);
      }
      else {
      intake.setMotor(Constants.IntakeConstants.CUBE_IN_SPEED * maintainSpeedFraction);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.subsystems.intake;

public interface IntakeIO {
    public void setMotor(double intakeSpeed);
    public double getCurrent();
    public double getEncoderSpeed();
}

package frc.robot.subsystems.intake;

public interface IntakeIO {
    public void setMotor(double speed);
    public double getCurrent();
    public double getEncoderSpeed();
    public double getEncoderPosition();
    public void setPosition(double position);
    public void setCurrentLimit(int current);
}

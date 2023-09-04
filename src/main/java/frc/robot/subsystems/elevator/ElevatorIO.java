package frc.robot.subsystems.elevator;

public interface ElevatorIO {       
    
    public double getEncoderPosition();
    public double getEncoderSpeed();
    public void setSpeed(double speed);
    public void setPosition(double position);
    public void periodicUpdate();
    public double getElevatorCurrent();
}

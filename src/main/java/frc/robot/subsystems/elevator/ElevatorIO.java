package frc.robot.subsystems.elevator;

public interface ElevatorIO {       
    
    public double getEncoderPosition();
    public double getEncoderSpeed();
    public void setSpeed(double speed);
    public void setPosition(double position);
    public void updateForSim();
    public double getElevatorCurrent();
    public boolean isAtUpperLimit();
    public boolean isAtLowerLimit();
}

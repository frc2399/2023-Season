package frc.robot.subsystems.elevator;

public interface ElevatorIO {       
    
    public double getEncoderPosition();
    public double getEncoderSpeed();
    public void setSpeed(double speed);
    public void updateForSim();

}

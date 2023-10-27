package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public interface ArmIO {
    public double getEncoderPosition();
    public double getEncoderSpeed();
    public void setSpeed(double speed);
    public void setPosition(double position);
    public void periodicUpdate();
    public double getArmCurrent();
    public void setSetpoint(State setpoint, double feedforward);
}

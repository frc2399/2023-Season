package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface ArmIO {
    public double getEncoderPosition();
    public double getEncoderSpeed();
    public void setSpeed(double speed);
    public void updateForSim();

    public void useOutput(double output, TrapezoidProfile.State setpoint);
    public double getMeasurement();
}

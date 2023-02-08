package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.SimEncoder;

public class SimElevator implements ElevatorIO {
    public static SimEncoder elevatorSimEncoder;
    public static ElevatorSim elevatorSim;
    
    @Override
    public double getEncoderSpeed() {
        return elevatorSimEncoder.getSpeed();
    }

    @Override
    public void setSpeed(double speed) {
        
        
    }

    @Override
    public double getEncoderPosition() {
        return elevatorSimEncoder.getDistance();
    }
}



    

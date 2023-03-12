package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimIntake implements IntakeIO{

    private DCMotorSim leftMotorSim;

    public SimIntake() {
        leftMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        leftMotorSim.setInput(intakeSpeed);
    }

     //Do not use these methods; they won't do much. 
     @Override
     public double getCurrent() {
        SmartDashboard.putNumber("Intake Current", SmartDashboard.getNumber("Intake Current", -100));
        return SmartDashboard.getNumber("Intake Current", -100);
    }
    
}

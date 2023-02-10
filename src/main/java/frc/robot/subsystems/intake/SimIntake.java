package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

public class SimIntake implements IntakeIO{

    private SolenoidSim leftSolenoidSim;
    private SolenoidSim rightSolenoidSim;
    private DCMotorSim leftMotorSim;
    private DCMotorSim rightMotorSim;

    public SimIntake() {
        REVPHSim simPH = new REVPHSim();
        leftSolenoidSim = new SolenoidSim(simPH, 1);
        rightSolenoidSim = new SolenoidSim(simPH, 2);
        leftMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
        rightMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        leftMotorSim.setInput(intakeSpeed);
        rightMotorSim.setInput(intakeSpeed);
    }
    
}

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SimEncoder;

public class SimIntake implements IntakeIO{

    private DCMotorSim intakeMotorSim;
    public static SimEncoder intakeSimEncoder;
    private double intakePower;

    public SimIntake() {
        intakeSimEncoder = new SimEncoder("intake");
        intakeMotorSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 1);
        SmartDashboard.putNumber("intake current sim", 0);
        SmartDashboard.putNumber("intake sim velocity", 0);
    }

    @Override
    public void setMotor(double intakeSpeed) {
        intakeMotorSim.setInput(intakeSpeed);
    }

     //Do not use these methods; they won't do much. 
     @Override
     public double getCurrent() {
        return SmartDashboard.getNumber("intake current sim", -100);
    }

    @Override
    public double getEncoderSpeed() {
        return SmartDashboard.getNumber("intake sim velocity", -100);
    }
    
}

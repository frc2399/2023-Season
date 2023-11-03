package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.SimEncoder;

public class SimArm implements ArmIO{

    private SimEncoder armEncoderSim;
    private SingleJointedArmSim armSim;
    private double armPower;

    public SimArm() {
        armEncoderSim = new SimEncoder("Elevator");
        armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1), // 1 NEO motor on the climber
                75,
                SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH, ArmConstants.ARM_MASS),
                ArmConstants.ARM_LENGTH,
                ArmConstants.MIN_ARM_ANGLE,
                ArmConstants.MAX_ARM_ANGLE,
                true);
    }

    @Override
    public double getEncoderPosition() {
        return armEncoderSim.getDistance();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoderSim.getSpeed();
    }

    @Override
    public void setSpeed(double speed) {
        armPower = speed;        
    }

    @Override
    public void setPosition(double position) {
        armEncoderSim.setDistance(position);
    }

    @Override
    public void periodicUpdate(){
        // sets input for elevator motor in simulation
        armSim.setInput(armPower * RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.02);
        // Finally, we set our simulated encoder's readings
        armEncoderSim.setDistance(armSim.getAngleRads());
        // sets our simulated encoder speeds
        armEncoderSim.setSpeed(armSim.getVelocityRadPerSec());

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    }

    @Override
    public double getArmCurrent() {
        return armSim.getCurrentDrawAmps();
    }

    @Override
    public void setSetpoint(State setpoint, double feedforward) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getMotorDutyCycle()
    {
        return 0.0; // TODO can we do better?
    };

}

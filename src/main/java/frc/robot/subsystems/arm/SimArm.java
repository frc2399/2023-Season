package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SimEncoder;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class SimArm implements ArmIO{

    private SimEncoder armEncoderSim;
    private SingleJointedArmSim armSim;
    private double armPower;
    private static double current_pos = 0;
    private static double current_vel = 0;

    public SimArm() {
        armEncoderSim = new SimEncoder("Elevator");
        armSim = new SingleJointedArmSim(
        DCMotor.getNEO(1), //1 NEO motor on the climber
        10, //TODO find out gearing
        SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH, ArmConstants.ARM_MASS), 
        ArmConstants.ARM_LENGTH,
        ArmConstants.MIN_ARM_ANGLE,
        ArmConstants.MAX_ARM_ANGLE,
        ArmConstants.ARM_MASS,
        true
      ); 
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
        SmartDashboard.putNumber("ArmSpeed", speed);
        
    }

    @Override
    public void updateForSim(){
        current_pos = armEncoderSim.getDistance();
        current_vel = armEncoderSim.getSpeed();
        SmartDashboard.putNumber("Arm Velocity", current_vel); 
        SmartDashboard.putNumber("Arm Postion", current_pos); 
        // sets input for elevator motor in simulation
        armSim.setInput(armPower * RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        armSim.update(0.02);
        // Finally, we set our simulated encoder's readings
        armEncoderSim.setDistance(armSim.getAngleRads());
        // sets our simulated encoder speeds
        armEncoderSim.setSpeed(armSim.getVelocityRadPerSec());

        SmartDashboard.putNumber("arm angle", armSim.getAngleRads());

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
        RobotContainer.armMechanism.setAngle(Units.radiansToDegrees(armSim.getAngleRads()) - 50);
    }


    
}
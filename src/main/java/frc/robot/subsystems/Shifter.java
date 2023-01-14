package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Shifter extends SubsystemBase {

	private DoubleSolenoid shifter;

	public Shifter() {
		/**
		 * Assigns Solenoids to correct PCM address and port
		 */
		shifter = new DoubleSolenoid(DriveConstants.PCM_ADDRESS, PneumaticsModuleType.CTREPCM, DriveConstants.SHIFT_HIGH_SPEED_SOLENOID_PCM_PORT, 
																							   DriveConstants.SHIFT_HIGH_TORQUE_SOLENOID_PCM_PORT);

	}
	
	public void setShifterHighSpeed() {
		shifter.set(Value.kReverse);

		DriveTrain.leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_SPEED_REVOLUTION_TO_INCH_CONVERSION);
        DriveTrain.rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_SPEED_REVOLUTION_TO_INCH_CONVERSION);
	}
	
	public void setShifterHighTorque() {
		shifter.set(Value.kForward);

		DriveTrain.leftEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
        DriveTrain.rightEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
	}

	public boolean isHighSpeed() {
		return shifter.get() == Value.kForward;
	}

} 
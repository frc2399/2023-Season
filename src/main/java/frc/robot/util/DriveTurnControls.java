package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.XboxConstants;
//import frc.robot.subsystems.Shifter;

public class DriveTurnControls {

    private Joystick xbox;
    private SlewRateLimiter driveLimiter;

    public DriveTurnControls(Joystick xBox) {
        this.xbox = xBox;
        double driveSlew = SmartDashboard.getNumber("drive slew", XboxConstants.DRIVE_SLEW_RATE);
        //double turnSlew = SmartDashboard.getNumber("turn slew", XboxConstants.TURN_SLEW_RATE);
        this.driveLimiter = new SlewRateLimiter(driveSlew);
        //this.turnLimiter = new SlewRateLimiter(turnSlew);
    }

    public double getDrive() {
        double val = xbox.getRawAxis(XboxConstants.ARCADE_DRIVE_SPEED_AXIS);

        // have deadband to prevent joystick drifting
        if (Math.abs(val) <= XboxConstants.FORWARD_DEADBAND) {
            val = 0;
        } 
        val = val * XboxConstants.FORWARD_JOYSTICK_INVERT;

        driveLimiter.calculate(val);

        // altering driving joystick sensitivity
        //double a = RobotContainer.a_value.getDouble(0.0);
        //val = ((1 - a) * val) + (a * Math.pow(val, 3));

        return val;
    }

    public double getTurn() {
        double val = xbox.getRawAxis(XboxConstants.ARCADE_DRIVE_TURN_AXIS);
        SmartDashboard.putNumber("value", val);
        if (Math.abs(val) <= XboxConstants.TURN_DEADBAND) {
            val = 0.0;
        }
        
        val = val * XboxConstants.TURN_JOYSTICK_INVERT;

        double a = DriveConstants.TURN_SENSITIVITY;
        val = ((1 - a) * val) + (a * Math.pow(val, 3));
     
        driveLimiter.calculate(val);
        // val = Math.pow(val, 3);

        val = val * DriveConstants.MAX_TURN_SPEED;
        SmartDashboard.putNumber("new value", val);
        
        return val;
    }

}
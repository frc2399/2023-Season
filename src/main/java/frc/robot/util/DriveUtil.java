package frc.robot.util;

public class DriveUtil {
    public static double computeDeadband(double x, double deadband) {
        if (Math.abs(x) <= deadband) { 
            x = 0; 
            return x; 
        }
        else {
            return x;
        }
    }
}

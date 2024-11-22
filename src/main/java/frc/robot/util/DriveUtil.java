package frc.robot.util;

public class DriveUtil {
    public static double computeDeadband(double x, double deadband) {
        if (Math.abs(x) <= deadband) { 
            return 0; 
        }
        else {
            return x;
        }
    }
}

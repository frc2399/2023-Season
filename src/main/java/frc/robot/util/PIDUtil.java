package frc.robot.util;

public class PIDUtil {
    
    // Checks if the current value is within the range of the setpoint being passed
  public static boolean checkWithinRange(double setpoint, double currentValue, double range) {
    return (Math.abs(currentValue - setpoint) < range);
  }
}

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class UIUtil {
    public static void setRumblePattern(int numRumbles, Joystick joystick) {
        try {
            for (int i = 0; i < numRumbles; i++) {
                joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
                Thread.sleep(250);
                joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                Thread.sleep(250);
            }
        } catch (InterruptedException e) {
        }
    }

}
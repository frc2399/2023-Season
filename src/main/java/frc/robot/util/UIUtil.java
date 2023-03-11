package frc.robot.util;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class UIUtil {
    public static void setRumblePattern(int numRumbles, Joystick joystick) {
        for (int i = 0; i < numRumbles; i++)
        {
            joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
            }

        }
    }

}

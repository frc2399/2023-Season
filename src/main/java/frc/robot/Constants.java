package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public static final int NEO550_CURRENT_LIMIT = 20;
    public static final int NEO_CURRENT_LIMIT = 60;

    public static final class DriveConstants {

        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1;
        public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 5;

        //motor ids
        public static final int RIGHT_FRONT_DRIVE_CAN_ID = 3;
        public static final int RIGHT_BACK_DRIVE_CAN_ID = 4;
        public static final int LEFT_FRONT_DRIVE_CAN_ID = 1;
        public static final int LEFT_BACK_DRIVE_CAN_ID = 2;

        // encoder 
        public static final double ENCODER_CALIBRATION_METERS = 0.0493;

        // No turning sensitivity
        public static final double MAX_TURN_SPEED = 1;

        public static final double TURN_SENSITIVITY = 0;

        public static final double SLOW_SPEED_FRACTION = 0.3;


        //path planning constants
        public static final double ks = 0.12704;
        public static final double kv = 2.4165;
        public static final double ka = 0.46642;
        public static final double kPDriveVel = 0.16761;
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
          new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class ElevatorConstants {

        public static final int LEFT_ELEVATOR_MOTOR_ID = 7;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 10;
        public static final int ELEVATOR_SLEW = 5;
        public static final double MIN_ELEVATOR_HEIGHT = 0.05;
        public static final double MAX_ELEVATOR_HEIGHT = 0.75;

        public static final double CONE_TOP_HEIGHT = 0.69;
        public static final double CUBE_TOP_HEIGHT = 0.58;
        public static final double CONE_MID_HEIGHT = 0.34;
        public static final double CUBE_MID_HEIGHT = 0.25;
        public static final double CONE_LOW_HEIGHT = 0.08;
        public static final double CUBE_LOW_HEIGHT = 0.08;

        public static final double CONE_UP_INTAKE_HEIGHT = 0.08;
        // public static final double CONE_TIP_INTAKE_HEIGHT= xx;
        // public static final double CONE_PHALANGE_INTAKE_HEIGHT = xx;
        // public static final double CUBE_INTAKE_HEGHT = xx;
        public static final double CONE_SHELF_INTAKE_HEIGHT = 0.705;
        public static final double CUBE_SHELF_INTAKE_HEIGHT = 0.508;

        // 27 inches, 41.951946 encoder counts
        public static final double ENCODER_CALIBRATION_METERS = Units.inchesToMeters(27) / 41.951946;

        public static final double HEIGHT_TOLERANCE = Units.inchesToMeters(1);

    }
    public static final class ArmConstants {
        //Arm angle in radians
        public static final double MAX_ARM_ANGLE = Math.PI/4;
        public static final double MIN_ARM_ANGLE = -Math.PI/4 * 3;
        //arm Mass in kg
        public static final double ARM_MASS = 2;
        //arm length in meters
        public static final double ARM_LENGTH = 0.3;

        public static final int ARM_MOTOR_ID = 8;

        public static final double CONE_TOP_ANGLE = 0.09;
        public static final double CUBE_TOP_ANGLE = 0.06;
        public static final double CONE_MID_ANGLE = 0.29;
        public static final double CUBE_MID_ANGLE = 0.46;
        public static final double CONE_LOW_ANGLE = 0.03;
        public static final double CUBE_LOW_ANGLE = 0.03;

        public static final double CONE_UP_INTAKE_ANGLE = -0.24;
        // public static final double CONE_TIP_INTAKE_ANGLE = xx;
        // public static final double CONE_PHALANGE_INTAKE_ANGLE = xx;
        // public static final double CUBE_INTAKE_ANGLE = xx;
        public static final double CONE_SHELF_INTAKE_ANGLE = -0.160;
        public static final double CUBE_SHELF_INTAKE_ANGLE = 0.308;

        public static final double RADIANS_PER_REVOLUTION = 0.0837;
        public static final double INITIAL_OFFSET = 0.711;

        public static final double ANGLE_TOLERANCE = Units.degreesToRadians(2);
        // Arm PID constants
        

    }

    public static final class IntakeConstants {

        public static final int LEFT_INTAKE_MOTOR_ID = 5;
        //public static final int RIGHT_INTAKE_MOTOR_ID = 10;
        public static final int PCM_CAN_ID = 3;
        //public static final int FORWARD_CHANNEL_SOLENOID_ID = 2;
        //public static final int REVERSE_CHANNEL_SOLENOID_ID = 3;
        public static final int EXTEND_INTAKE_ARM_RIGHT = 2;
        public static final double INTAKE_SLEW_RATE = 10;
        public static final double CONE_IN_SPEED = 1.0;
        public static final double CONE_OUT_SPEED = -1.0;
        public static final double CUBE_IN_SPEED = -1.0;
        public static final double CUBE_OUT_SPEED = 1.0;
    }

    public static final class LEDConstants {
        public static final int RED_CHANNEL = 0;
        public static final int GREEN_CHANNEL = 1;
        public static final int BLUE_CHANNEL = 2;
        public static final int[] blue2399 = {0, 100, 255};
        public static final int[] pink2399 = {255, 50, 200};
        public static final double WAIT_TIME = 1.0;

    }


    public static final class JoystickConstants {
        // joystick ports
        //public static final int JOYSTICK_PORT = 1;
    } 

    public static final class XboxConstants {
        public static final int XBOX_DRIVER_PORT = 0;
        public static final int XBOX_OPERATOR_PORT = 0;

        public static final int FORWARD_JOYSTICK_INVERT = 1;
        public static final int TURN_JOYSTICK_INVERT = -1;

        public static final double FORWARD_DEADBAND = 0.05;
        public static final double TURN_DEADBAND = 0.1;

        public static final double TURN_SLEW_RATE = 5.0;

        public static final double JOYSTICK_SENSITIVITY = 0.5;
    }

}
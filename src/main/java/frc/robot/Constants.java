package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static final class DriveConstants {

        //motor ids
        public static final int RIGHT_FRONT_DRIVE_CAN_ID = 1;
        public static final int RIGHT_BACK_DRIVE_CAN_ID = 2;
        public static final int LEFT_FRONT_DRIVE_CAN_ID = 4;
        public static final int LEFT_BACK_DRIVE_CAN_ID = 3;

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
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 8;
        public static final int ELEVATOR_SLEW = 5;
        public static final double MIN_ELEVATOR_HEIGHT = Units.inchesToMeters(2);
        public static final double MAX_ELEVATOR_HEIGHT = Units.inchesToMeters(75);
        
    }

    //TODO add real coordinates
    public static final class LimelightConstants {
        public static final Transform3d APRILTAG_CAMERA_TO_ROBOT =
            new Transform3d(new Translation3d(Units.inchesToMeters(14), Units.inchesToMeters(34), Units.inchesToMeters(0)), 
            new Rotation3d(Units.degreesToRadians(-90.0), 0, Units.degreesToRadians(-90.0))); 
        public static final Transform3d APRILTAG_ROBOT_TO_CAMERA = APRILTAG_CAMERA_TO_ROBOT.inverse();
        public static final double CAMERA_PITCH_RADIANS = 0;
        public static final double CAMERA_HEIGHT_METERS = 1;
        public static final double TARGET_HEIGHT_METERS = 1;
    }

    public static final class ArmConstants {
        //Arm angle in radians
        public static final double MAX_ARM_ANGLE = Math.PI/4;
        public static final double MIN_ARM_ANGLE = -Math.PI/4 * 3;
        //arm Mass in kg
        public static final double ARM_MASS = 1;
        //arm length in meters
        public static final double ARM_LENGTH = 0.3;
        public static final int ARM_MOTOR_ID = 5;
        public static final double RADIANS_PER_REVOLUTION = 2 * Math.PI;
    }

    public static final class IntakeConstants {

        public static final int LEFT_INTAKE_MOTOR_ID = 9;
        public static final int RIGHT_INTAKE_MOTOR_ID = 10;
        public static final int PCM_CAN_ID = 3;
        public static final int FORWARD_CHANNEL_SOLENOID_ID = 2;
        public static final int REVERSE_CHANNEL_SOLENOID_ID = 3;
        public static final int EXTEND_INTAKE_ARM_RIGHT = 2;
        public static final double INTAKE_SLEW_RATE = 10;
        public static final double INTAKE_IN_SPEED = 1.0;
        public static final double INTAKE_OUT_SPEED = -1.0;

    }


    public static final class JoystickConstants {
        // joystick ports
        public static final int JOYSTICK_PORT = 1;

    } 

    public static final class XboxConstants {
        public static final int XBOX_PORT = 0;

        public static final int FORWARD_JOYSTICK_INVERT = 1;
        public static final int TURN_JOYSTICK_INVERT = -1;

        public static final double FORWARD_DEADBAND = 0.05;
        public static final double TURN_DEADBAND = 0.1;

        public static final double DRIVE_SLEW_RATE = 5.0;
        public static final double TURN_SLEW_RATE = 5.0;

        public static final double JOYSTICK_SENSITIVITY = 0.5;
    }
}
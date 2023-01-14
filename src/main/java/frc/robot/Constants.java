package frc.robot;


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
        // motor ids
        public static final int RIGHT_FRONT_DRIVE_CAN_ID = 3;
        public static final int RIGHT_MIDDLE_DRIVE_CAN_ID = 4;
        public static final int RIGHT_BACK_DRIVE_CAN_ID = 5;
        public static final int LEFT_FRONT_DRIVE_CAN_ID = 6;
        public static final int LEFT_MIDDLE_DRIVE_CAN_ID = 7;
        public static final int LEFT_BACK_DRIVE_CAN_ID = 8;

        //solenoids
        public static final int SHIFT_HIGH_SPEED_SOLENOID_PCM_PORT = 2;
	    public static final int SHIFT_HIGH_TORQUE_SOLENOID_PCM_PORT = 3;
        public static final int PCM_ADDRESS = 0;
        public static final double SHIFT_TIMER = 0.5;

        //follow vision target cmd speeds
        public static final double TARGET_FOLLOWING_SPEED = 0.5;
        public static final double BUTTERY_FOLLOWING_SPEED = 0.035;

        // encoder 
       public static final double HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION = 2.317175948;
       public static final double HIGH_SPEED_REVOLUTION_TO_INCH_CONVERSION = 1.048247093;


        public static final double TURN_TO_HUB_KP = 0.005;

        // No turning sensitivity
        public static final double MAX_TURN_SPEED = 1;

        public static final double TURN_SENSITIVITY = 0;

        public static final double SLOW_SPEED_FRACTION = 0.3;

    }

    public static final class JoystickConstants {
        // joystick ports
        public static final int JOYSTICK_PORT = 1;

        public static final int FORWARD_JOYSTICK_INVERT = 1;    
        public static final int TURN_JOYSTICK_INVERT = 1;
      



        public static final int SHOOTER_BTN = 3; 

        public static final int CLIMBER_UP = 5;
        public static final int CLIMBER_DOWN = 3;

        public static final int RIGHT_CLIMBER_UP = 4;
        public static final int LEFT_CLIMBER_UP = 6;

        public static final int MAX_SHOOT = 7;

        public static final int TURN_TO_HUB = 9;

    } 

    public static final class XboxMappingToJoystick{
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int RIGHT_STICK_X = 4;
        public static final int RIGHT_STICK_Y = 5;

        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;

        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;
        public static final int START_BUTTON = 8;
        public static final int BACK_BUTTON = 7;

        public static final int LEFT_STICK_PUSH = 9;
        public static final int RIGHT_STICK_PUSH = 10;
    }

    public static final class XboxConstants {
        public static final int XBOX_PORT = 0;

        public static final int ARCADE_DRIVE_SPEED_AXIS = XboxMappingToJoystick.LEFT_STICK_Y;
        public static final int ARCADE_DRIVE_TURN_AXIS = XboxMappingToJoystick.RIGHT_STICK_X; 


        public static final int FORWARD_JOYSTICK_INVERT = 1;
        public static final int TURN_JOYSTICK_INVERT = 1;

        
        public static final int SHIFT_HIGH_SPEED = XboxMappingToJoystick.RIGHT_BUMPER; 
        public static final int SHIFT_HIGH_TORQUE = XboxMappingToJoystick.LEFT_BUMPER ; 
        public static final int INTAKE = XboxMappingToJoystick.LEFT_TRIGGER; 
        public static final int INDEX_AND_SHOOT = XboxMappingToJoystick.RIGHT_TRIGGER;
        public static final int POINT_AND_SHOOT = XboxMappingToJoystick.A_BUTTON;
        public static final int SPIT_OUT_BALL = XboxMappingToJoystick.Y_BUTTON;



        public static final double FORWARD_DEADBAND = 0.05;
        public static final double TURN_DEADBAND = 0.1;

        public static final double DRIVE_SLEW_RATE = 5.0;
        public static final double TURN_SLEW_RATE = 5.0;

        public static final double JOYSTICK_SENSITIVITY = 0.5;
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 1;
        public static final double INTAKESPEED = 0.75;
        public static final double INTAKE_SLEW = 5;

        //intake arm
        public static final int EXTEND_INTAKE_ARM = 1;
        public static final int RETRACT_INTAKE_ARM = 0;
    }

    public static final class ShooterConstants {
        public static final int PID_IDX = 0;
        public static final int CAN_TIMEOUT = 10;
        public static final int TOP_MOTOR_ID = 10; 
        public static final int BOTTOM_MOTOR_ID = 9;

        public static final double SHOOTER_SPEED = 0.06;
 
        public static final double SETPOINT_RATIO = 2.7;
        public static final double BOTTOM_SETPOINT = 2100;
        public static final double TOP_SETPOINT = BOTTOM_SETPOINT * SETPOINT_RATIO;
        public static final double MAX_SETPOINT = 3000;

        public static final double FENDER_LOWER_SHOOTER_TOP_SPEED = 900;
        public static final double FENDER_LOWER_SHOOTER_BOTTOM_SPEED = 850;

        // public static final double FENDER_LOWER_SHOOTER_TOP_SPEED = 1100;
        // public static final double FENDER_LOWER_SHOOTER_BOTTOM_SPEED = 650;

        public static final double FENDER_UPPER_SHOOTER_TOP_SPEED = 575 * 1.1;
        public static final double FENDER_UPPER_SHOOTER_BOTTOM_SPEED = 2875 * 1.1;

        public static final double TARMAC_UPPER_SHOOTER_TOP_SPEED = 1198;
        public static final double TARMAC_UPPER_SHOOTER_BOTTOM_SPEED = 2995;

        public static final double MIDDLE_AUTON_TARMAC_UPPER_SHOOTER_TOP_SPEED = 907.5;
        public static final double MIDDLE_AUTON_TARMAC_UPPER_SHOOTER_BOTTOM_SPEED = 3025;

        public static final double SHOOTER_KP = .0003;//1.875;
        public static final double SHOOTER_KI = 0.000001;//0.006;
        // public static final double SHOOTER_KI = 1;//0.006;
        //0.000001
        public static final double SHOOTER_KD = 0.0003;//52.5;
        public static final double SHOOTER_KF = 0.00023; //0.15;
        public static final double SHOOTER_KIZ = 250;
        // public static final double SHOOTER_KIZ = 0;

        //250
        public static final double SHOOTER_K_MAX_OUTPUT = 1;
        public static final double SHOOTER_K_MIN_OUTPUT = 0;
        public static final double SHOOTER_MAX_RPM = 5700;
    
        public static final double SHOOTER_MAX_ACCEL = 0;
    }

    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 2;
        public static final double INDEXERSPEED = 0.5;
        public static final int LIMIT_SWITCH_ID = 0;
        
        public static final int INDEXER_SLEW = 5;
    }

    // public static final class PrototypeDriveConstants {
    //     public static final int LEFT_FRONT_MOTOR_ID = 8;
    //     public static final int LEFT_MIDDLE_MOTOR_ID = 7;
    //     public static final int LEFT_BACK_MOTOR_ID = 6;

    // }
  
    public static final class ClimberConstants {
        public static final int LEFT_CLIMBER_MOTOR_ID = 12;
        public static final int RIGHT_CLIMBER_MOTOR_ID = 11;
        public static final double CLIMBER_SPEED = 0.5;
        
        public static final double MAX_HEIGHT = 24;
        public static final double MIN_HEIGHT = 0;

        public static final int CLIMBER_SLEW = 5;
        
        public static final int EXTEND_PISTON = 4;
        public static final int RETRACT_PISTON = 5;


    }


    public static final class PhotonLimelightConstants {
        public static final double CAMERA_HEIGHT_INCHES = 38;
        public static final double TARGET_HEIGHT_INCHES = 104;
        public static final double HUB_RADIUS_INCHES = 27;
        public static final double TILT_DEGREES = 36;
        // public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
        // public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);

        public static final double ANGLE_ERROR_TOLERANCE = 3; 
    }
}
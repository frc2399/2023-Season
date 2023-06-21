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

    public static final int NEO550_CURRENT_LIMIT = 30;
    public static final int NEO_CURRENT_LIMIT = 60;

    public static final class DriveConstants {

        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 5;

        //motor ids
        public static final int RIGHT_FRONT_DRIVE_CAN_ID = 3;
        public static final int RIGHT_BACK_DRIVE_CAN_ID = 4;
        public static final int LEFT_FRONT_DRIVE_CAN_ID = 1;
        public static final int LEFT_BACK_DRIVE_CAN_ID = 2;

        //encoder 
        public static final double ENCODER_CALIBRATION_METERS = 0.0493;

        //no turning sensitivity
        public static final double MAX_TURN_SPEED = 1;
        public static final double TURN_SENSITIVITY = 0;
        public static final double SLOW_SPEED_FRACTION = 0.3;

        //path planning constants
        public static final double ks = 0.12704;
        public static final double kv = 2.4165;
        public static final double ka = 0.46642;
        public static final double kPDriveVel = 0.16761;
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        
        //reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class ElevatorConstants {

        public static final int LEFT_ELEVATOR_MOTOR_ID = 7;
        public static final int RIGHT_ELEVATOR_MOTOR_ID = 10;
        public static final int ELEVATOR_SLEW = 5;

        //elevator min and max heights in meters
        public static final double MIN_ELEVATOR_HEIGHT = 0.0;
        public static final double MAX_ELEVATOR_HEIGHT = 0.75;

        //elevatr heights for cone/cube for scoring low, mid, and top nodes
        public static final double CONE_LOW_HEIGHT = 0.09;
        public static final double CUBE_LOW_HEIGHT = 0.20;
        public static final double CONE_MID_HEIGHT = 0.58;
        public static final double CUBE_MID_HEIGHT = 0.48;
        public static final double CONE_TOP_HEIGHT = 0.71;
        public static final double CUBE_TOP_HEIGHT = 0.74;

        //elevator heights for intaking upright cones, cones from tip, cubes from ground and cone/cube from shelf
        public static final double CONE_UP_INTAKE_HEIGHT = 0.18;
        public static final double CONE_TIP_INTAKE_HEIGHT = 0.017;
        public static final double CUBE_INTAKE_HEIGHT = 0.06;
        public static final double CONE_SHELF_INTAKE_HEIGHT = 0.727;
        public static final double CUBE_SHELF_INTAKE_HEIGHT = 0.463;

        //27 inches per 41.951946 encoder counts
        public static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;

        //can be 1 inch off from goal setpoints and still considered at goal; made higher so placeConeOnNode cmd in auton will execute
        public static final double HEIGHT_TOLERANCE = Units.inchesToMeters(1);

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

        public static final int ARM_MOTOR_ID = 8;

        //arm min and max angles in radians
        public static final double MAX_ARM_ANGLE = Math.PI/4;
        public static final double MIN_ARM_ANGLE = -Math.PI/4 * 3;
    
        //arm mass in kg
        public static final double ARM_MASS = 2.72155;
        //arm length in meters
        public static final double ARM_LENGTH = 0.65;

        //arm angles for cone/cube for scoring low, mid, and top nodes
        public static final double CONE_LOW_ANGLE = 0.03;
        public static final double CUBE_LOW_ANGLE = 0.37;
        //-0.34
        public static final double CONE_MID_ANGLE = -0.37;
        public static final double CUBE_MID_ANGLE = -0.06;
        public static final double CONE_TOP_ANGLE = -0.25;
        public static final double CUBE_TOP_ANGLE = 0.07;

        //arm angles for intaking upright cones, cones from tip, cubes from ground and cone/cube from shelf
        public static final double CONE_UP_INTAKE_ANGLE = -0.56;
        public static final double CONE_TIP_INTAKE_ANGLE = -0.50;
        public static final double CUBE_INTAKE_ANGLE = -0.36;
        public static final double CONE_SHELF_INTAKE_ANGLE = -0.302;
        public static final double CUBE_SHELF_INTAKE_ANGLE = 0.308;

        public static final double TURTLE_ANGLE = 0.5;

        public static final double SAD_ANGLE = -Math.PI/4 * 2;

        public static final double HAPPY_ANGLE = Math.PI/8;

        public static final double RADIANS_PER_REVOLUTION = 0.0837;
        // initial offset is 0.711 + (0.287) - (0.308)
        public static final double INITIAL_OFFSET = 0.660;

        //can be 2 degrees off from goal setpoints and still considered at goal; made higher so arm.atGoal() in placeConeOnNode cmd will execute in auton
        public static final double ANGLE_TOLERANCE_AUTON = Units.degreesToRadians(2);

    }

    public static final class IntakeConstants {

        public static final int INTAKE_MOTOR_ID = 5;
        public static final double INTAKE_SLEW_RATE = 10;
        public static final double CONE_IN_SPEED = -1.0;
        public static final double CONE_OUT_SPEED = 1.0;
        public static final double CUBE_IN_SPEED = 0.6;
        public static final double CUBE_OUT_SPEED = -1.0;
        public static final int CONE_IN_CURRENT = 30;
        public static final int CUBE_IN_CURRENT = 25;
        public static final int OUT_CURRENT = 30;
    }

    public static final class LEDConstants {
        public static final int RED_CHANNEL = 2;
        public static final int GREEN_CHANNEL = 1;
        public static final int BLUE_CHANNEL = 3;
        public static final int WHITE_CHANNEL = 0;
        public static final int[] blue2399 = {0, 100, 255};
        public static final int[] pink2399 = {255, 50, 200};
        public static final double WAIT_TIME = 1.0;

    }

    public static final class XboxConstants {
        public static final int XBOX_DRIVER_PORT = 0;
        public static final int XBOX_OPERATOR_PORT = 1;

        public static final int FORWARD_JOYSTICK_INVERT = 1;
        public static final int TURN_JOYSTICK_INVERT = -1;

        public static final double FORWARD_DEADBAND = 0.05;
        public static final double TURN_DEADBAND = 0.1;

        public static final double TURN_SLEW_RATE = 5.0;

        public static final double JOYSTICK_SENSITIVITY = 0.5;
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
        public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);
    }

    public static final class DanceConstants {
        public static final double DANCE_INTAKE_SPEED = 0.3; 
        public static final double DANCE_SLOW_SPEED_FRACTION = 0.2;
        //multiplied this value by maximum possible error (2pi) to get desired speed of 0.5 
        public static final double kP = 0.08; 
        public static final double ANGLE_ERROR_TOLERANCE = Units.degreesToRadians(3);
        public static final double DRIVE_FWD_TIME = 4.0;
        public static final double DRIVE_FWD_SPD_LIMIT = 1.0;
        //This is in RADIANS
        public static double PIROUETTE_ANGLE = Units.degreesToRadians(540.0);
        //fiddle with this as needed. untested. 
        public static double PIROUETTE_kP = 0.1;

        
    }

}
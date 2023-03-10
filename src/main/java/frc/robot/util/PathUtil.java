package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class PathUtil {

    public PathUtil() {

    }

    public static Pose2d getInitialPoseForAlliance(PathPlannerTrajectory trajectory) {
        Pose2d pose = trajectory.getInitialPose();
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            Rotation2d rotation = pose.getRotation();
            Translation2d translation = pose.getTranslation();

            Translation2d transformedTranslation =
            new Translation2d(translation.getX(), Constants.FieldConstants.FIELD_WIDTH_METERS - pose.getY());
            Rotation2d transformedHeading = rotation.times(-1);

            return new Pose2d(transformedTranslation, transformedHeading);
        }
        else {
            return pose;
        }
    }
    
}

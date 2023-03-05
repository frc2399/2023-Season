package frc.robot.subsystems.limelight;

import java.io.IOException;

import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class SimLimelight extends SubsystemBase{
    // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    double camPitch = LimelightConstants.CAMERA_PITCH_RADIANS; // degrees
    double camHeightOffGround = LimelightConstants.CAMERA_HEIGHT_METERS; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels
    private DriveTrain driveTrain;
    AprilTagFieldLayout fieldLayout;

    SimVisionSystem simVision =
            new SimVisionSystem(
                    "photonvision",
                    camDiagFOV,
                    new Transform3d(
                            new Translation3d(0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0)),
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 208
    double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    double tgtXPos = Units.feetToMeters(54);
    double tgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);

    Pose3d farTargetPose =
        new Pose3d(
            new Translation3d(tgtXPos, tgtYPos, LimelightConstants.TARGET_HEIGHT_METERS),
            new Rotation3d(0.0, 0.0, 0.0));

    public SimLimelight(DriveTrain driveTrain)
    {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        this.driveTrain = driveTrain;
        simVision.addSimVisionTarget(new SimVisionTarget(farTargetPose, targetWidth, targetHeight, 0));
        for(int i = 1; i <= 8; i++)
        {
            
            DriveTrain.field.getObject("target" + i).setPose(fieldLayout.getTagPose(i).get().toPose2d());

        }
    }
    @Override
    public void periodic() {
        simVision.processFrame(driveTrain.getPoseMeters());
    }
}

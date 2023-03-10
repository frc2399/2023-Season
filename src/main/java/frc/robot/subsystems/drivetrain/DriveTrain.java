// Maisie was HERE HAHAHHAHAHAHHAHA :D
// Shrividya was here :))
// Alice was here ;3
// Ethan was here
// John was here (\/)(;;)(\/)
// Clare was here!!!!!!!!!!!!!!
// Anna (the better one) was here :)
// Anna (the even better one) was here :)

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NavX.AHRS;

public class DriveTrain extends SubsystemBase {

    private DriveIO driveIO;

    public AHRS ahrs;
    public static PIDController turnController;

    private double pitchRate;

    public static double outputSpeed;

    private LinearFilter derivativeCalculator = LinearFilter.backwardFiniteDifference(1, 2, 0.02);


    // simulation
    private DifferentialDriveOdometry odometry;

    
    public static Field2d field = new Field2d();

    public DriveTrain(DriveIO io) {
        driveIO = io;

        // this code is instantiating the simulated sensors and actuators when the robot is in simulation
        
        odometry = new DifferentialDriveOdometry(getGyroAngle(), getLeftEncoderMeters(), getRightEncoderMeters(), new Pose2d(9, 6.5, new Rotation2d(3.14/2)));

        SmartDashboard.putData("Field", field);

        field.setRobotPose(new Pose2d(9, 6.5, new Rotation2d(3.14/2)));

    }

    @Override
    public void periodic() {

        // runs sim periodic code in simDrive
        driveIO.updateForSim();
        driveIO.updateForReal();

        odometry.update(
            // we want CCW positive, CW negative
            getGyroAngle(),
            getLeftEncoderMeters(),
            getRightEncoderMeters()
        
        );


        pitchRate = derivativeCalculator.calculate(getGyroPitch());

        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("gyro angle", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("left encoder postion", getLeftEncoderMeters());
        SmartDashboard.putNumber("right encoder postion", getRightEncoderMeters());
        SmartDashboard.putNumber("left encoder velocity", getLeftEncoderMetersPerSecond());
        SmartDashboard.putNumber("right encoder veloicty", getRightEncoderMetersPerSecond());
        SmartDashboard.putNumber("odometry x", getPoseMeters().getX());
        SmartDashboard.putNumber("odometry y", getPoseMeters().getY());
        SmartDashboard.putNumber("odometry angle", getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("pitch", getGyroPitch());
        SmartDashboard.putNumber("pitch rate", getGyroPitchRate());

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setMotors(double leftSpeed, double rightSpeed) {
        driveIO.setMotors(leftSpeed, rightSpeed);
    }

    public void setMotorVoltage(double leftVolt, double rightVolt) {
        driveIO.setMotorVoltage(leftVolt, rightVolt);
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeedsMetersPerSecond() {        
        return new DifferentialDriveWheelSpeeds(getLeftEncoderMetersPerSecond(), getRightEncoderMetersPerSecond());
    }

    public Pose2d getPoseMeters(){
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
    }
    
    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistanceMeters() {
        return (getLeftEncoderMeters() + getRightEncoderMeters()) / 2.0;
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */

    public double getGyroPitch() {
        return driveIO.getGyroPitch();
    }

    public double getGyroPitchRate()
    {
        return pitchRate;
    }

    public Rotation2d getGyroAngle() {
        return driveIO.getGyroAngle(); 
    }

    public double getRightEncoderMeters() {
        return driveIO.getRightEncoderMeters();
    }

    public double getLeftEncoderMeters() {
            return driveIO.getLeftEncoderMeters();
        }


    public double getRightEncoderMetersPerSecond() {
            return driveIO.getRightEncoderMetersPerSecond();
        }

    public double getLeftEncoderMetersPerSecond() {
            return driveIO.getLeftEncoderMetersPerSecond();
        }

}

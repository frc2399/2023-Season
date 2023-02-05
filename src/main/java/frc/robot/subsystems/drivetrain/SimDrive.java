package frc.robot.subsystems.drivetrain;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SimEncoder;
import frc.robot.util.SimGyro;;


public class SimDrive implements DriveIO {


   // Basically copy over the entire rest of the drive subsytem that is "sim"
   // Remember to move code from simulationPeriodic to the new "updateForSim" method (see document)
   private double leftDrivePower, rightDrivePower;
   public SimEncoder leftEncoderSim;
   public SimEncoder rightEncoderSim;
   public SimGyro gyroSim;
   private DifferentialDrivetrainSim driveSim;
   public Field2d field = new Field2d();


   public SimDrive() {
    leftEncoderSim = new SimEncoder("Left Drive");
    rightEncoderSim = new SimEncoder("Right Drive");
    gyroSim = new SimGyro("NavX");

    // Create the simulation model of our drivetrain.
    driveSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(3),       // 3 NEO motors on each side of the drivetrain.
        8,                       // 8:1 gearing reduction. for now
        6,                       // MOI of 6 kg m^2 (from CAD model). for now
        Units.lbsToKilograms(140), // The mass of the robot is 140 lbs (with battery) which is 63 kg
        Units.inchesToMeters(2.1), // The robot uses 2.1" radius wheels.
        Units.inchesToMeters(27.811), // The track width is 27.811 inches.

        // The standard deviations for measurement noise:
        // x and y:          0 m
        // heading:          0 rad
        // l and r velocity: 0  m/s
        // l and r position: 0 m
        VecBuilder.fill(0, 0, 0, 0, 0, 0, 0)
        );

   }


   @Override

   public void setMotorVoltage(double leftVolt, double rightVolt) {
        leftDrivePower = leftVolt / 12; //RobotController.getInputVoltage());
        rightDrivePower = rightVolt / 12;
   }

   public void setMotors(double leftSpeed, double rightSpeed) {
        leftDrivePower = leftSpeed;
        rightDrivePower = rightSpeed;
    }

   public Rotation2d getGyroAngle() {
        return gyroSim.getAngle();
   }

   public double getRightEncoderMeters() {
       return rightEncoderSim.getDistance();
   }

   public double getLeftEncoderMeters() {
        return leftEncoderSim.getDistance();
   }

   public double getLeftEncoderMetersPerSecond() {
        return leftEncoderSim.getSpeed();
    }
    
    public double getRightEncoderMetersPerSecond() {
        return rightEncoderSim.getSpeed();
    }


    @Override
    public void updateForSim() {
        // This method will be called once per scheduler run when in simulation
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.

        driveSim.setInputs(
            leftDrivePower * RobotController.getInputVoltage(),
            rightDrivePower * RobotController.getInputVoltage()
        );
    
        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        driveSim.update(0.02);

        // Update all of our sensors.
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setSpeed(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setSpeed(driveSim.getRightVelocityMetersPerSecond());

        // we want CCW positive, CW negative
        gyroSim.setAngle(new Rotation2d(driveSim.getHeading().getRadians()));
        
    }
}

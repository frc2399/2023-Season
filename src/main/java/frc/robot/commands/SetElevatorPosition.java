package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


public class SetElevatorPosition extends CommandBase {
    private static final double elevatorHighSetpoint = Units.inchesToMeters(70);
    public static final double elevatorLowSetpoint = Units.inchesToMeters(5);
    public static double elevatorSetpoint;
    public static DoublePublisher elevatorSetpointPublisher;
    public static DoublePublisher elevatorTargetPosPublisher;
    public static DoublePublisher elevatorTargetVelPublisher;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("datatable");

     // publish to the topic in "datatable" called "Out"
   DoublePublisher elevatorPositionPublisher = datatable.getDoubleTopic("elevator Pos").publish();
   DoublePublisher elevatorVelocityPublisher = datatable.getDoubleTopic("elevator Vel").publish();
   

   public static EncoderSim elevatorEncoderSim;
   public static Encoder elevatorEncoder;
   public static ElevatorSim elevatorSim;
   public final static PWMSparkMax motorController = new PWMSparkMax(0);
  public final static Joystick elevatorJoystick = new Joystick(0);

  
    // private static final double feedForward = 0.55;
    // private static final double kpPos = 1;
    // private static final double kpVel = 0.01;
  
    // tuned values:
    private static final double feedForward = 0.55;
    private static final double kpPos = 2;
    private static final double kpVel = .2;
  
    private static double current_pos = 0;
    private static double current_vel = 0;
    private static double gravityCompensation = .075;
   
  
   //private static final PIDController pidController = new PIDController(1.5, .006, 0.006);
  
    // Trapezoidal profile constants and variables
    private static final double max_vel = 1.0;  // m/s
    private static final double max_accel = 1.0;  // m/s/s
  
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(max_vel, max_accel);
    TrapezoidProfile.State previousProfiledReference = new TrapezoidProfile.State(0.0, 0.0);
  
  

    private final Elevator elevator;
    private final double height;


    public SetElevatorPosition (Elevator elevator, double height) {
        elevatorSetpointPublisher = datatable.getDoubleTopic("elevator setpoint").publish();
   elevatorTargetPosPublisher = datatable.getDoubleTopic("Target Pos").publish();
   elevatorTargetVelPublisher = datatable.getDoubleTopic("Target Vel").publish();
        this.elevator = elevator;
        this.height = height;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
  
        //this code is instantiating the simulator stuff for climber
    if(RobotBase.isSimulation()) {
        Encoder encoder = new Encoder(2,3);
        Elevator.elevatorEncoderSim = new EncoderSim(encoder);
          // final Joystick elevatorJoystick = new Joystick(0);
          Elevator.elevatorSim = new ElevatorSim(
            Elevator.elevatorGearbox, 
            Elevator.elevatorGearRatio,
            Elevator.elevatorCarriageMass, 
            Elevator.elevatorDrumRadius,
            Elevator. minElevatorHeight,
            Elevator.maxElevatorHeight,
            true,
            VecBuilder.fill(0.01)
  
          ); 
  
          Elevator.elevatorEncoder.setPositionConversionFactor(Constants.DriveConstants.HIGH_TORQUE_REVOLUTION_TO_INCH_CONVERSION);
        System.out.println("Set elevator position started");
    }
    }

    @Override
    public void execute() {
    elevatorPositionPublisher.set( elevatorEncoderSim.getDistance());
    elevatorVelocityPublisher.set( elevatorEncoderSim.getRate());
    current_pos = elevatorEncoderSim.getDistance();
    current_vel = elevatorEncoderSim.getRate();
    elevatorSetpointPublisher.set(elevatorSetpoint);
    //sets input for elevator motor in simulation
    elevatorSim.setInput(motorController.get() * RobotController.getBatteryVoltage());
    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);
    // Finally, we set our simulated encoder's readings
    elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
    // elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
    //sets our simulated encoder speeds
    elevatorEncoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));


    // Read joystick and control elevator
    if (elevatorJoystick.getRawButton(1)) {
      elevatorSetpoint = elevatorHighSetpoint;
      System.out.println("HIGH setpoint!!!");

    } else if (elevatorJoystick.getRawButton(2)) {
      elevatorSetpoint = elevatorLowSetpoint;
      System.out.println("LOW Setpoint!!!!!");
    }

    


    SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);

    // double speed = pidController.calculate(elevatorEncoderSim.getDistance(), elevatorSetpoint);

   

    // Update the profile each timestep to get the current target position and velocity
    TrapezoidProfile.State referenceSetpoint = new TrapezoidProfile.State(elevatorSetpoint, 0.0);
    TrapezoidProfile profile = new TrapezoidProfile(constraints, referenceSetpoint, previousProfiledReference);
    previousProfiledReference = profile.calculate(0.02); // Assumes 50Hz loop. Could measure this directly

    double target_pos = previousProfiledReference.position;
    double target_vel = previousProfiledReference.velocity;

    elevatorTargetPosPublisher.set(target_pos);
    elevatorTargetVelPublisher.set(target_vel);

    double speed = feedForward * target_vel + kpPos * (target_pos - current_pos) + kpVel * (target_vel - current_vel);
    speed = speed + gravityCompensation;
    motorController.set(speed);


    

   

    // double speed = Elevator.elevatorJoystick.getRawAxis(0);
     Elevator.motorController.set(speed); 


  
        this.elevator.setSpeed(speed);
    
        System.out.println("Climber Speed from Execute" + speed);


    //     if (m_climber.isLeftExtended()){
    //     this.m_climber.setLeftSpeed(0);   
    //    }
    //    else {
    //     this.m_climber.setLeftSpeed(speed);
    //    };

    //    if (m_climber.isRightExtended()){
    //     this.m_climber.setRightSpeed(0);   
    //    }
    //    else {
    //     this.m_climber.setRightSpeed(speed);
    //    };
    }

    @Override
    public void end(boolean interrupted) {
        this.elevator.setSpeed(0);
    
    }

    @Override
    public boolean isFinished() {
        // if (m_climber.isLeftExtended() && m_climber.isRightExtended()) {
        //     return true;
        // }
       return false;
    }
}


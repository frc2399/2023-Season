package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ExtendElevator extends CommandBase {

    private final Elevator m_elevator;
    private final double speed;


    public ExtendElevator (Elevator elevator, double speed) {
        this.m_elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("Extend climber started");
    
    }

    @Override
    public void execute() {
        this.m_elevator.setSpeed(speed);
    
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
        this.m_elevator.setSpeed(0);
    
    }

    @Override
    public boolean isFinished() {
        // if (m_climber.isLeftExtended() && m_climber.isRightExtended()) {
        //     return true;
        // }
       return false;
    }
}


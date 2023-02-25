// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.LEDController;

public class LED extends SubsystemBase {
  LEDController red = new LEDController(Constants.LEDConstants.RED_CHANNEL);
  LEDController green= new LEDController(Constants.LEDConstants.GREEN_CHANNEL);
  LEDController blue = new LEDController(Constants.LEDConstants.BLUE_CHANNEL);
  Timer timer = new Timer();


  boolean isPink = true;

  /** Creates a new LED. */
  public LED() {
    timer.reset();
    timer.start();
  }

public void setColor(int r, int g, int b) {
  red.set(r);
  green.set(g);
  blue.set(b);
  RobotContainer.LEDMechanism.setColor(new Color8Bit(r, g, b));
}



  @Override
  public void periodic() {
    // if(timer.get() > Constants.LEDConstants.WAIT_TIME)
    // {
    //   if(isPink)
    //   {
    //     this.setColor(Constants.LEDConstants.blue2399[0], 
    //     Constants.LEDConstants.blue2399[1], 
    //     Constants.LEDConstants.blue2399[2]);
    //   }
    //   else {
    //     this.setColor(Constants.LEDConstants.pink2399[0], 
    //     Constants.LEDConstants.pink2399[1], 
    //     Constants.LEDConstants.pink2399[2]);
    //   }
    //   isPink = !isPink;
    //   timer.reset();
    //   timer.start();
    // }
    
    if(RobotContainer.coneMode == true) {
      this.setColor(240, 200, 50);
    }
    else {
      this.setColor(150, 30, 150);
    }

  }
}

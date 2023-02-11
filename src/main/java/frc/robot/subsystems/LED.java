// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDController;

public class LED extends SubsystemBase {
  LEDController red = new LEDController(0);
  LEDController green= new LEDController(1);
  LEDController blue = new LEDController(2);
  Timer timer = new Timer();


  boolean isPink = true;

  /** Creates a new LED. */
  public LED() {
    timer.reset();
    timer.start();


  }
private void setPink(){
  red.set(50);
  green.set(255);
  blue.set(39);

}
private void setBlue() {
  red.set(255);
  green.set(100);
  blue.set(0);
}



  @Override
  public void periodic() {
    if(timer.get() > 1.0 )
    {
      if(isPink)
      {
        this.setBlue();
      }
      else {
        this.setPink();
      }
      isPink = !isPink;
      timer.reset();
      timer.start();
    }
  }
}

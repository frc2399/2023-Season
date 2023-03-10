// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {

  private PhotonCamera photonCamera;
  /** Creates a new Camera. */
  public Camera(PhotonCamera photonCamera) {
    this.photonCamera = photonCamera;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var pipelineResult = photonCamera.getLatestResult();

    if(pipelineResult.hasTargets())
    {
      //System.out.println(pipelineResult.getBestTarget().getYaw());
    }
  }
}

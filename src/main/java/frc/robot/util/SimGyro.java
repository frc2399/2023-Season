package frc.robot.util;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
public class SimGyro {

  private SimDouble angle; 

  /**
   * Construct a sim device.
   *
   * @param name Name of the gyro. Must be different from any other SimGyro
   */
  public SimGyro(String name) {
    SimDevice device = SimDevice.create("Gyro[" + name + "]");
    angle = device.createDouble("Angle", Direction.kOutput, 0);
  }
  
  /**
   * Set the SimGyro's angle.
   *
   * @param angle Rotation2d representing the angle.
   */
  public void setAngle(Rotation2d angle) {
    this.angle.set(angle.getRadians());
  }

  /**
   * Get the angle of the SimGyro.
   *
   * @return Current angle of the SimGyro as a Rotation2d
   */
  public Rotation2d getAngle() {
    return new Rotation2d(angle.get());
  }
}
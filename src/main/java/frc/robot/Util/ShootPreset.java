package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShootPreset {
  private Rotation2d hoodAngle;
  private AngularVelocity flywheelVelocity;

  public ShootPreset(Rotation2d hoodAngle, AngularVelocity flywheeVelocity) {
    this.hoodAngle = hoodAngle;
    this.flywheelVelocity = flywheeVelocity;
  }

  public Rotation2d getHoodAngle() {
    return hoodAngle;
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheelVelocity;
  }
}

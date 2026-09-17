package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class TestShooterWrapper {
  public AngularVelocity flywheelVelo;
  public Rotation2d hoodAngle;

  public TestShooterWrapper(AngularVelocity flywheel, Rotation2d hoodAngle) {
    this.hoodAngle = hoodAngle;
    this.flywheelVelo = flywheel;
  }
}

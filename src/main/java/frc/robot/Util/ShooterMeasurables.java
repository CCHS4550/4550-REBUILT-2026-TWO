package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterMeasurables {
  private boolean isValid;
  private Rotation2d driveAngle;
  private double driveVelocity;
  private double hoodAngle;
  private double hoodVelocity;
  private double flywheelSpeed;
  private double distance;
  private double distanceNoLookahead;
  private double timeOfflight;
  private boolean passing;

  public ShooterMeasurables(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {
    this.isValid = isValid;
    this.driveAngle = driveAngle;
    this.driveVelocity = driveVelocity;
    this.hoodAngle = hoodAngle;
    this.hoodVelocity = hoodVelocity;
    this.flywheelSpeed = flywheelSpeed;
    this.distance = distance;
    this.distanceNoLookahead = distanceNoLookahead;
    this.timeOfflight = timeOfFlight;
    this.passing = passing;
  }

  public boolean getIsValid() {
    return isValid;
  }

  public Rotation2d getDriveAngle() {
    return driveAngle;
  }

  public double getDriveVelocity() {
    return driveVelocity;
  }

  public double getHoodAngle() {
    return hoodAngle;
  }

  public double getHoodVelocity() {
    return hoodVelocity;
  }

  public double getFlywheelSpeed() {
    return flywheelSpeed;
  }

  public double getDistance() {
    return distance;
  }

  public double getDistanceNoLookahead() {
    return distanceNoLookahead;
  }

  public double getTimeOfFlight() {
    return timeOfflight;
  }

  public boolean getPassing() {
    return passing;
  }
}

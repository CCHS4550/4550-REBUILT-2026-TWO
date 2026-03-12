package frc.robot.Config;

public class ShooterConfig {

  public double elevationKp;
  public double elevationKi;
  public double elevationKd;
  public double elevationKs;
  public double elevationKv;

  public double shooterKp;
  public double shooterKi;
  public double shooterKd;
  public double shooterKs;
  public double shooterKv;

  

  public ShooterConfig withElevationKp(double elevationKp) {
    this.elevationKp = elevationKp;
    return this;
  }

  public ShooterConfig withElevationKi(double elevationKi) {
    this.elevationKi = elevationKi;
    return this;
  }

  public ShooterConfig withElevationKd(double elevationKd) {
    this.elevationKd = elevationKd;
    return this;
  }

  public ShooterConfig withElevationKs(double elevationKs) {
    this.elevationKs = elevationKs;
    return this;
  }

  public ShooterConfig withElevationKv(double elevationKv) {
    this.elevationKv = elevationKv;
    return this;
  }

  public ShooterConfig withShooterKp(double shooterKp) {
    this.shooterKp = shooterKp;
    return this;
  }

  public ShooterConfig withShooterKi(double shooterKi) {
    this.shooterKi = shooterKi;
    return this;
  }

  public ShooterConfig withShooterKd(double shooterKd) {
    this.shooterKd = shooterKd;
    return this;
  }

  public ShooterConfig withShooterKs(double shooterKs) {
    this.shooterKs = shooterKs;
    return this;
  }

  public ShooterConfig withShooterKv(double shooterKv) {
    this.shooterKv = shooterKv;
    return this;
  }
}

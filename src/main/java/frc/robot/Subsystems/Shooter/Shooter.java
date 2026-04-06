package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIO;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIOInputsAutoLogged;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.Util.ShooterMeasurables;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private FlywheelIO flywheelIO;
  private ElevationIO elevationIO;

  private ElevationIOInputsAutoLogged elevationInputs = new ElevationIOInputsAutoLogged();
  private FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private boolean atGoal;

  private ShooterMeasurables wantedShooterMeasurables =
      new ShooterMeasurables(false, new Rotation2d(), 0, 0, 0, 0, 0, 0, 0, false);

  public enum ShooterSystemState {
    IDLE,
    GOTO_WANTED_MEASURABLES,
    ZERO,
    TEST
  }

  public enum ShooterWantedState {
    IDLE,
    ACTIVE_SHOOT,
    ZERO,
    TEST
  }

  private ShooterSystemState systemState = ShooterSystemState.IDLE;
  private ShooterWantedState wantedState = ShooterWantedState.IDLE;

  public Shooter(ElevationIO elevationIO, FlywheelIO flywheelIO) {
    this.elevationIO = elevationIO;
    this.flywheelIO = flywheelIO;
    atGoal = false;
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Subsystems/flywheels", flywheelInputs);

    elevationIO.updateInputs(elevationInputs);
    Logger.processInputs("Subsystems/elevation", elevationInputs);

    atGoal = atSetpoint();

    systemState = handleStateTransitions();
    applyStates();
  }

  public ShooterSystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return ShooterSystemState.IDLE;
      case ACTIVE_SHOOT:
        return ShooterSystemState.GOTO_WANTED_MEASURABLES;
      case ZERO:
        return ShooterSystemState.ZERO;
      case TEST:
        return ShooterSystemState.TEST;
      default:
        return ShooterSystemState.IDLE;
    }
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        elevationIO.setVoltage(-0.3);
        flywheelIO.setVoltage(Voltage.ofBaseUnits(0, Volts));
        break;
      case GOTO_WANTED_MEASURABLES:
        setElevationAngle(Rotation2d.fromRadians(wantedShooterMeasurables.getHoodAngle()));
        setFlywheelSpeed(RadiansPerSecond.of(wantedShooterMeasurables.getFlywheelSpeed()));
        break;
      case ZERO:
        setElevationAngle(
            Rotation2d.fromRadians(
                Constants.ShooterConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS));
        setFlywheelSpeed(RadiansPerSecond.of(0));
        break;
      case TEST:
        setFlywheelSpeed(RadiansPerSecond.of(200));
        elevationIO.setVoltage(-0.3);
        break;
    }
  }

  private void setElevationAngle(Rotation2d angle) {
    elevationIO.setElevationAngle(angle);
  }

  private void setFlywheelSpeed(AngularVelocity velo) {
    flywheelIO.setVelo(velo);
  }

  public void setShooterMeasurables(ShooterMeasurables shooterMeasurables) {
    this.wantedShooterMeasurables = shooterMeasurables;
  }

  public void setWantedState(ShooterWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public ShooterSystemState getSystemState() {
    return systemState;
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(
            flywheelInputs.flywheel1VelocityRadPerSec,
            wantedShooterMeasurables.getFlywheelSpeed(),
            wantedShooterMeasurables.getFlywheelSpeed()
                * 0.05) // Allowance is 5% of wanted velocity
        && MathUtil.isNear(
            elevationInputs.elevationAngle.getRadians(),
            wantedShooterMeasurables.getHoodAngle(),
            0.3);
  }
}

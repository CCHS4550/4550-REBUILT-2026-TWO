package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  private CommandXboxController controller;

  private boolean atGoal;
  public boolean testing;

  private ShooterMeasurables wantedShooterMeasurables =
      new ShooterMeasurables(false, new Rotation2d(), 0, 0, 0, 0, 0, 0, 0, false);

  public enum ShooterSystemState {
    IDLE,
    GOTO_WANTED_MEASURABLES,
    MANUAL_SHOOT,
    ZERO,
    TEST
  }

  public enum ShooterWantedState {
    IDLE,
    ACTIVE_SHOOT,
    MANUAL_SHOOT,
    ZERO,
    TEST
  }

  private ShooterSystemState systemState = ShooterSystemState.IDLE;
  private ShooterWantedState wantedState = ShooterWantedState.IDLE;

  public Shooter(ElevationIO elevationIO, FlywheelIO flywheelIO, CommandXboxController controller) {
    this.elevationIO = elevationIO;
    this.flywheelIO = flywheelIO;
    this.controller = controller;
    atGoal = false;
    testing = false;
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Subsystems/flywheels", flywheelInputs);

    elevationIO.updateInputs(elevationInputs);
    Logger.processInputs("Subsystems/elevation", elevationInputs);

    atGoal = atSetpoint();

    if (!testing) {
      systemState = handleStateTransitions();
      applyStates();
    }
  }

  public ShooterSystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return ShooterSystemState.IDLE;
      case ACTIVE_SHOOT:
        return ShooterSystemState.GOTO_WANTED_MEASURABLES;
        // case MANUAL_SHOOT:
        //   return ShooterSystemState.MANUAL_SHOOT;
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
        // case MANUAL_SHOOT:
        //   double elevationInput = controller.getRightY();
        //   elevationInput = MathUtil.applyDeadband(elevationInput, 0.1);

        //   double currentAngle = elevationInputs.elevationAngle.getRadians();
        //   double adjustmentRate = 1.5; // hopefully is in rad/sec

        //   double newAngle = currentAngle + elevationInput * adjustmentRate * 0.02;

        //   newAngle =
        //       MathUtil.clamp(
        //           newAngle,
        //           Constants.ShooterConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS,
        //           Constants.ShooterConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);

        //   setElevationAngle(Rotation2d.fromRadians(newAngle));
        //   if (wantedFlywheelState) {
        //     setFlywheelSpeed(RadiansPerSecond.of(200));
        //   }

        //   break;
      case ZERO:
        setElevationAngle(
            Rotation2d.fromRadians(
                Constants.ShooterConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS));
        setFlywheelSpeed(RadiansPerSecond.of(0));
        break;
      case TEST:
        // setFlywheelSpeed(RadiansPerSecond.of(200));
        setFlywheelVoltage(2);
        break;
    }
  }

  private void setElevationAngle(Rotation2d angle) {
    elevationIO.setElevationAngle(angle);
  }

  private void setFlywheelSpeed(AngularVelocity velo) {
    flywheelIO.setVelo(velo);
  }

  public void setFlywheelVoltage(double voltage) {
    flywheelIO.setVoltage(Voltage.ofBaseUnits(voltage, Volt));
  }

  public void testPIDFWithValues(double targetRPM, double kP, double kS, double kV) {
    double feedforward = (kV * targetRPM) + kS;
    double error = targetRPM - flywheelInputs.flywheel1VelocityRadPerSec;
    double feedback = error * kP;

    setFlywheelVoltage(feedback + feedforward);
    testing = true;
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

  public void adjustFlywheelKSlotValue(double value, String slot) {
    flywheelIO.adjustFlywheelKSlotValue(value, slot);
  }

  public void adjustElevationKSlotValue(double value, String slot) {
    elevationIO.adjustElevationKSlotValue(value, slot);
  }
}

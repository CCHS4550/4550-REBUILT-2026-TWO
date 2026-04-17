package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constant.Constants;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIO;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIOInputsAutoLogged;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.Util.ShooterMeasurables;
import frc.robot.Util.TestShooterWrapper;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private Timer timer = new Timer();
  private final LinearFilter veloFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private double shooterInitVelo = 375.5;
  public double passingVelo = 377.5;

  private double smoothedSpeedRadPerSec = 0.0;
  


  private FlywheelIO flywheelIO;
  private ElevationIO elevationIO;

  private ElevationIOInputsAutoLogged elevationInputs = new ElevationIOInputsAutoLogged();
  private FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private CommandXboxController controller;

  private boolean atGoal;

  private ShooterMeasurables wantedShooterMeasurables =
      new ShooterMeasurables(false, new Rotation2d(), 0, 0, 0, 0, 0, 0, 0, false);

  public enum ShooterSystemState {
    IDLE,
    GOTO_WANTED_MEASURABLES,
    MANUAL_SHOOT,
    ZERO,
    TEST,
    TEST_2,
    PASSING,
    TEST_INTERP_MEASURABLES
  }

  public enum ShooterWantedState {
    IDLE,
    ACTIVE_SHOOT,
    MANUAL_SHOOT,
    ZERO,
    TEST,
    TEST_2,
    PASSING,
    TEST_INTERP_MEASURABLES
  }

  private ShooterSystemState systemState = ShooterSystemState.IDLE;
  private ShooterWantedState wantedState = ShooterWantedState.IDLE;

  public Shooter(ElevationIO elevationIO, FlywheelIO flywheelIO, CommandXboxController controller) {
    this.elevationIO = elevationIO;
    this.flywheelIO = flywheelIO;
    this.controller = controller;
    atGoal = false;
  }

  @Override
  public void periodic() {
    smoothedSpeedRadPerSec = veloFilter.calculate(flywheelInputs.flywheel1VelocityRadPerSec);

    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Subsystems/flywheels", flywheelInputs);

    elevationIO.updateInputs(elevationInputs);
    Logger.processInputs("Subsystems/elevation", elevationInputs);

    atGoal = atSetpoint();

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
      case TEST_2:
        return ShooterSystemState.TEST_2;
      case PASSING:
      return ShooterSystemState.PASSING;
      case TEST_INTERP_MEASURABLES:
        return ShooterSystemState.TEST_INTERP_MEASURABLES;
      default:
        return ShooterSystemState.IDLE;
    }
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        elevationIO.setVoltage(0.0);
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
        flywheelIO.setVelo(AngularVelocity.ofBaseUnits(shooterInitVelo, RadiansPerSecond));
        setElevationAngle(Rotation2d.fromDegrees(61));
        break;
      case TEST_2:
        flywheelIO.setVelo(AngularVelocity.ofBaseUnits(377.9, RadiansPerSecond));
        setElevationAngle(Rotation2d.fromDegrees(61));
        break;
      case PASSING:
        flywheelIO.setVelo(AngularVelocity.ofBaseUnits(passingVelo, RadiansPerSecond));
        setElevationAngle(Rotation2d.fromDegrees(61));
        break;
      default:
        break;
    }
  }

  public void setElevationAngle(Rotation2d angle) {
    elevationIO.setElevationAngle(angle);
  }

  public void setFlywheelSpeed(AngularVelocity velo) {
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

    System.out.println("kP: " + kP + " kS: " + kS + " kV: " + kV);
  }

  public void setShooterSpeed(double rpm) {
    flywheelIO.setSpeed(rpm);
  }

  public void setShooterMeasurables(ShooterMeasurables shooterMeasurables) {
    this.wantedShooterMeasurables = shooterMeasurables;
  }

  public void setWantedState(ShooterWantedState wantedState) {
    if(wantedState == this.wantedState){
      return;
    }

    this.wantedState = wantedState;
  }

  public ShooterSystemState getSystemState() {
    return systemState;
  }

  public boolean atSetpoint() {
    double speed = shooterInitVelo;
    if(systemState == ShooterSystemState.PASSING) {
      speed = passingVelo;
    }

    if(systemState != ShooterSystemState.TEST && systemState != ShooterSystemState.TEST_2){
      timer.reset();
      return false;
    }

    boolean close = MathUtil.isNear(
            flywheelInputs.flywheel1VelocityRadPerSec,
            speed,
            speed
                * 0.06) // Allowance is 6% of wanted velocity
        && MathUtil.isNear(
            elevationInputs.elevationAngle.getRadians(),
            1.064,
            0.3);
    if(close && !timer.isRunning()){
      timer.start();
    }

    if(close && timer.hasElapsed(0.5)){
      return true;
    }
    else{
      return false;
    }
  }

  public void adjustFlywheelKSlotValue(double value, String slot) {
    flywheelIO.adjustFlywheelKSlotValue(value, slot);
  }

  public void adjustElevationKSlotValue(double value, String slot) {
    elevationIO.adjustElevationKSlotValue(value, slot);
  }
}

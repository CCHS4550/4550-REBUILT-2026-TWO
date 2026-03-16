package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIO;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.Util.LaunchCalculator;
import frc.robot.Util.ShooterMeasurables;

public class Shooter extends SubsystemBase {

  private FlywheelIO flywheelIO;
  private ElevationIO elevationIO;

  private LaunchCalculator calculator;

  private ElevationIOInputsAutoLogged elevationInputs = new ElevationIOInputsAutoLogged();
  private FlywheelIOInputsAutoLogged flywheelInputs = new ElevationIOInputsAutoLogged();

  private double goalVelocity;
  private double goalAngle;

  private boolean atGoal;
  private boolean isPassing;
  private boolean isReadyToShoot;
  
  private ShooterMeasurables wantedShooterMeasurables;
  private AngularVelocity desiredSpeed;
  private Rotation2d desiredHoodAngle;

  public enum DesiredTarget {
    HUB, 
    PASSING,
    NONE
  }

  private DesiredTarget desiredTarget = DesiredTarget.NONE;

  public enum ShooterSystemState {
    IDLE,
    ACTIVE_SHOOT_HUB,
    ACTIVE_SHOOT_PASSING,
    TRACKING_TARGET_HUB,
    TRACKING_TARGET_PASSING,
    ZERO
  }

  public enum ShooterWantedState {
    IDLE,
    ACTIVE_SHOOT,
    TRACKING_TARGET,
    ZERO
  }

  private ShooterSystemState systemState = ShooterSystemState.IDLE;
  private ShooterWantedState wantedState = ShooterWantedState.IDLE;

  public Shooter(ElevationIO elevationIO, FlywheelIO flywheelIO, LaunchCalculator calculator) {
    this.elevationIO = elevationIO;
    this.flywheelIO = flywheelIO;
    this.calculator = calculator;

    goalVelocity = 0.0;
    goalAngle = 0.0;

    isPassing = false;
    atGoal = false;
  }


  @Override
  public void periodic (){
    wantedShooterMeasurables = calculator.getParameters();
    isPassing = wantedShooterMeasurables.getPassing();

    desiredHoodAngle = Rotation2d.fromRadians(wantedShooterMeasurables.getHoodAngle());
    desiredSpeed = RadiansPerSecond.of(wantedShooterMeasurables.getFlywheelSpeed());

    atGoal = atSetpoint();

    systemState = handleStateTransitions();
    applyStates();



  }

  public ShooterSystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return ShooterSystemState.IDLE;
        break;
      case ACTIVE_SHOOT:
        if (isPassing && wantedShooterMeasurables.getIsValid()){
            return ShooterSystemState.ACTIVE_SHOOT_PASSING;
        }
        else if (!isPassing && wantedShooterMeasurables.getIsValid()){
            return ShooterSystemState.ACTIVE_SHOOT_HUB;
        }
        else if (isPassing && !wantedShooterMeasurables.getIsValid()){
            return ShooterSystemState.TRACKING_TARGET_PASSING;
        }
        else {
            return ShooterSystemState.TRACKING_TARGET_HUB;
        }
        
        break;
      case TRACKING_TARGET:
        if (isPassing){
            return ShooterSystemState.TRACKING_TARGET_PASSING;
        }
        else {
            return ShooterSystemState.TRACKING_TARGET_HUB;
        }
        break;
      case ZERO:
        return ShooterSystemState.ZERO;
        break;
      default:
        return ShooterSystemState.IDLE;
        break;
    }
  }

  public void applyStates() {
    switch (systemState){
        case IDLE:
            elevationIO.setVoltage(0);
            setFlywheelSpeed(RadiansPerSecond.of(0));
            desiredTarget = DesiredTarget.NONE;
            break;
        case ACTIVE_SHOOT_HUB:
            setElevationAngle(desiredHoodAngle);
            setFlywheelSpeed(desiredSpeed);
            desiredTarget = DesiredTarget.HUB;
            break;
        case ACTIVE_SHOOT_PASSING:
            setElevationAngle(desiredHoodAngle);
            setFlywheelSpeed(desiredSpeed);
            desiredTarget = DesiredTarget.PASSING;
            break;
        case TRACKING_TARGET_HUB:
            setElevationAngle(desiredHoodAngle);
            setFlywheelSpeed(RadiansPerSecond.of(0));
            desiredTarget = DesiredTarget.HUB;
            break;
        case TRACKING_TARGET_PASSING:
            setElevationAngle(desiredHoodAngle);
            setFlywheelSpeed(RadiansPerSecond.of(0));
            desiredTarget = DesiredTarget.PASSING;
            break;
        case ZERO:
            setElevationAngle(Rotation2d.fromDegrees(0));
            setFlywheelSpeed(RadiansPerSecond.of(0));
            desiredTarget = DesiredTarget.NONE;
            break;


    }
  }

  public void setElevationAngle(Rotation2d angle) {
    elevationIO.setElevationAngle(angle);
  }

  public void setFlywheelSpeed(AngularVelocity velo) {
    flywheelIO.setVelo(velo);
  }

  public boolean atSetpoint (){
    return MathUtil.isNear(flywheelInputs.flywheelVelocityRadPerSec, goalVelocity, 1)
           && MathUtil.isNear(elevationInputs.elevationAngle, goalAngle, 0.3);
  }
}

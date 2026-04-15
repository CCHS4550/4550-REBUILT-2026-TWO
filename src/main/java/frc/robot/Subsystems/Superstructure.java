package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.WantedState;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.Indexer.IndexerWantedState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.WantedIntakeState;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterSystemState;
import frc.robot.Subsystems.Shooter.Shooter.ShooterWantedState;
import frc.robot.Util.LaunchCalculator;
import frc.robot.Util.ShooterMeasurables;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;

  @AutoLogOutput private WantedSuperstructureState wantedState1 = WantedSuperstructureState.IDLE;
  @AutoLogOutput private SystemState systemState = SystemState.IDLE;

  private ShooterMeasurables shooterCalcs =
      new ShooterMeasurables(false, new Rotation2d(), 0, 0, 0, 0, 0, 0, 0, false);

  public Superstructure(
      SwerveSubsystem swerveSubsystem, Intake intake, Shooter shooter, Indexer indexer) {
    this.swerveSubsystem = swerveSubsystem;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    // Log launching parameters TODO: fix logging bugs later
    var launchCalculator = LaunchCalculator.getInstance();

    shooterCalcs = launchCalculator.getParameters();
    shooter.setShooterMeasurables(shooterCalcs);

    systemState = handleStateTransitions();
    applyStates();

    // Clear launching parameters
    launchCalculator.clearLaunchingParameters();
  }

  public void setWantedSuperstructureState(WantedSuperstructureState wantedState) {
    if (!DriverStation.isAutonomous()
        && wantedState != WantedSuperstructureState.SHOOT
        && swerveSubsystem.getSystemState()
            != frc.robot.Subsystems.Drive.SwerveSubsystem.SystemState.DRIVE_TO_POINT) {
      swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE);
    }

    if (DriverStation.isAutonomous()) {
      if (wantedState != WantedSuperstructureState.SHOOT) {
        if (swerveSubsystem.getSystemState()
                != frc.robot.Subsystems.Drive.SwerveSubsystem.SystemState.CHOREO_PATH
            && swerveSubsystem.getSystemState()
                != frc.robot.Subsystems.Drive.SwerveSubsystem.SystemState.DRIVE_TO_POINT) {
          swerveSubsystem.setWantedState(WantedState.IDLE);
        }
      }
    }

    // swervedrive state has been automatically reset for safety, but this will be overrun if we are
    // in auto aim
    this.wantedState1 = wantedState;
  }

  private void applyStates() {
    switch (systemState) {
      case IDLE:
        if (DriverStation.isDisabled()) {
          swerveSubsystem.setWantedState(WantedState.IDLE);
        }
        intake.setWantedIntakeState(WantedIntakeState.IDLE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.IDLE);
        break;
      case STOW:
        intake.setWantedIntakeState(WantedIntakeState.STOWED);
        indexer.setWantedState(IndexerWantedState.IDLE);
        if (shooter.getSystemState() != ShooterSystemState.ZERO) {
          shooter.setWantedState(ShooterWantedState.IDLE);
        }
        break;
      case ZERO:
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.ZERO);
        break;
      case EXTEND_INTAKE:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        if (shooter.getSystemState() != ShooterSystemState.ZERO) {
          shooter.setWantedState(ShooterWantedState.IDLE);
        }
        break;
      case INTAKING:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
        indexer.setWantedState(IndexerWantedState.IDLE);
        if (shooter.getSystemState() != ShooterSystemState.ZERO) {
          shooter.setWantedState(ShooterWantedState.IDLE);
        }
        break;
      case INTAKING_PRE_AIM:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.ACTIVE_SHOOT);
        break;
      case PASSIVE_PRE_AIM:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.ACTIVE_SHOOT);
        break;
      case AIMING:
        swerveSubsystem.setTargetRotation(shooterCalcs.getDriveAngle());
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.ACTIVE_SHOOT);
        break;
      case SHOOT:
        swerveSubsystem.setTargetRotation(shooterCalcs.getDriveAngle());
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        indexer.setWantedState(IndexerWantedState.RUNNING);
        shooter.setWantedState(ShooterWantedState.ACTIVE_SHOOT);
        break;
    }
  }

  private SystemState handleStateTransitions() {
    if (wantedState1 == WantedSuperstructureState.SHOOT && !shooterCalcs.getIsValid()) {
      return SystemState.IDLE;
    }

    switch (wantedState1) {
      case IDLE:
        return SystemState.IDLE;
      case STOW:
        return SystemState.STOW;
      case ZERO:
        return SystemState.ZERO;
      case EXTEND_INTAKE:
        return SystemState.EXTEND_INTAKE;
      case INTAKING:
        return SystemState.INTAKING;
      case SHOOT:
        if (shooter.atSetpoint() && swerveSubsystem.isAtDesiredRotation(0.2)) {
          return SystemState.SHOOT;
        }
        return SystemState.AIMING;
      case PRE_AIM:
        return SystemState.PASSIVE_PRE_AIM;
      case PRE_AIM_INTAKING:
        return SystemState.INTAKING_PRE_AIM;
      default:
        return SystemState.IDLE;
    }
  }

  public enum WantedSuperstructureState {
    IDLE,
    STOW,
    ZERO,
    EXTEND_INTAKE,
    INTAKING,
    SHOOT,
    PRE_AIM,
    PRE_AIM_INTAKING
  }

  private enum SystemState {
    IDLE,
    STOW,
    ZERO,
    EXTEND_INTAKE,
    INTAKING,
    INTAKING_PRE_AIM,
    PASSIVE_PRE_AIM,
    AIMING,
    SHOOT
  }
}

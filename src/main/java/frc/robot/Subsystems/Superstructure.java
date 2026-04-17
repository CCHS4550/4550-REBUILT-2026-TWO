package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants;
import frc.robot.Constant.FieldConstants;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.WantedState;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.Indexer.IndexerWantedState;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.WantedIntakeState;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterSystemState;
import frc.robot.Subsystems.Shooter.Shooter.ShooterWantedState;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.LaunchCalculator;
import frc.robot.Util.ShooterMeasurables;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private double shootingDisMeters = 2.25 + 0.349 + 1.016;

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

    passingFlywheelSpeedMap.put(3.0, 377.5);
    passingFlywheelSpeedMap.put(5.62, 450.0);
  }

  @Override
  public void periodic() {
    // Log launching parameters TODO: fix logging bugs later
    var launchCalculator = LaunchCalculator.getInstance();

    shooterCalcs = launchCalculator.getParameters();
    shooter.setShooterMeasurables(shooterCalcs);
    shooter.passingVelo = getLauncherVeloPassing();

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
        if (DriverStation.isAutonomous()) {
          swerveSubsystem.setWantedState(WantedState.IDLE);
        }
        if (!DriverStation.isAutonomous()) {
          swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE);
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
        shooter.setWantedState(ShooterWantedState.TEST);
        break;
      case PASSIVE_PRE_AIM:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.TEST);
        break;
      case AIMING:
        if (isPassing()) {
          swerveSubsystem.setTargetRotation(
              getDriveAngleWithLauncherOffset(
                  Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry(),
                  FieldConstants.getPassingPose().getTranslation()));
          intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
          indexer.setWantedState(IndexerWantedState.IDLE);
          shooter.setWantedState(ShooterWantedState.PASSING);
          break;
        }

        swerveSubsystem.setDesiredPoseForDriveToPoint(calculateLaunchPose(), 13);
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        indexer.setWantedState(IndexerWantedState.IDLE);
        shooter.setWantedState(ShooterWantedState.TEST);
        break;
      case SHOOT:
        if (isPassing()) {
          swerveSubsystem.setTargetRotation(
              getDriveAngleWithLauncherOffset(
                  Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry(),
                  FieldConstants.getPassingPose().getTranslation()));
          intake.setWantedIntakeState(WantedIntakeState.PUMPING);
          indexer.setWantedState(IndexerWantedState.RUNNING);
          shooter.setWantedState(ShooterWantedState.PASSING);
          break;
        }

        swerveSubsystem.setDesiredPoseForDriveToPoint(calculateLaunchPose(), 13);
        intake.setWantedIntakeState(WantedIntakeState.PUMPING);
        indexer.setWantedState(IndexerWantedState.RUNNING);
        shooter.setWantedState(ShooterWantedState.TEST_2);
        break;
    }
  }

  private SystemState handleStateTransitions() {
    // if (wantedState1 == WantedSuperstructureState.SHOOT && !shooterCalcs.getIsValid()) {
    //   return SystemState.IDLE;
    // }

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
        if (shooter.atSetpoint()
            && swerveSubsystem.isAtDesiredRotation(0.2)
            && swerveSubsystem.isAtDriveToPointSetpoint()) {
          return SystemState.SHOOT;
        }
        if (shooter.atSetpoint() && swerveSubsystem.isAtDesiredRotation(0.2) && isPassing()) {
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

  private Pose2d calculateLaunchPose() {

    Translation2d launch = new Translation2d();
    Translation2d x =
        Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getTranslation();
    Translation2d y = FieldConstants.getScoringPose().getTranslation();

    Translation2d direction = x.minus(y);
    double distance = direction.getNorm();

    if (distance == 0) {
      launch = x;
    }

    Translation2d unit =
        new Translation2d(direction.getX() / distance, direction.getY() / distance);
    launch = y.plus(unit.times(shootingDisMeters));

    Rotation2d angle =
        getDriveAngleWithLauncherOffset(
            Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry(),
            FieldConstants.getScoringPose().getTranslation());

    Pose2d launchPose = new Pose2d(launch, angle);
    return launchPose;
  }

  private Rotation2d getDriveAngleWithLauncherOffset(Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    Constants.ShooterConstants.SHOOTER_TRANSFORM.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle =
        fieldToHubAngle
            .plus(hubAngle)
            .plus(Constants.ShooterConstants.SHOOTER_TRANSFORM.getRotation().toRotation2d());
    return driveAngle;
  }

  private boolean isPassing() {
    boolean passing =
        AllianceFlipUtil.applyX(
                Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getX())
            > FieldConstants.LinesVertical.hubCenter;
    return passing;
  }

  private double getLauncherVeloPassing() {
    double distance =
        Robotstate.getInstance()
            .getRobotPoseFromSwerveDriveOdometry()
            .getTranslation()
            .getDistance(FieldConstants.getPassingPose().getTranslation());
    double velo = passingFlywheelSpeedMap.get(distance);
    if (velo > 600) {
      velo = 600;
    }
    return velo;
  }
}

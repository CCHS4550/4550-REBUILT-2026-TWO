package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants;
import frc.robot.Globals;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.Agitator.Agitator.WantedAgitatorState;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.WantedState;
import frc.robot.Subsystems.Intake.*;
import frc.robot.Subsystems.Intake.Intake.WantedIntakeState;
import frc.robot.Subsystems.Kicker.Kicker;
import frc.robot.Subsystems.Kicker.Kicker.KickerWantedState;
import frc.robot.Subsystems.Shooter.*;
import frc.robot.Subsystems.Shooter.Shooter.ShooterState;
import frc.robot.tools.ShotCalculator;
import frc.robot.tools.ShotCalculator.ShotSolution;
import frc.robot.tools.TunableNumber;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SwerveSubsystem drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Kicker kicker;
  private final Agitator agitator;
  double outakeIdleInitTime = 0;
  boolean outakeIdleInit = false;
  boolean firstTimeDefault = true;
  private SuperState lastState = SuperState.IDLE;
  private SuperState tempLastState = SuperState.IDLE;
  private ArrayList<Translation3d> trajectoryPoint = new ArrayList<Translation3d>();
  private ArrayList<Translation3d> trajectoryVelocity = new ArrayList<Translation3d>();
  private TunableNumber manualShootRPM = new TunableNumber("Manual Shoot RPM", 2000);
  private TunableNumber manualShootHoodAngle = new TunableNumber("Manual Shoot Hood Angle", 60.0);
  private TunableNumber manualShootTurretAngle =
      new TunableNumber("Manual Shoot Turret Angle", 0.0);
  private ShotSolution presetShotSolution =
      new ShotSolution(
          new Rotation2d(Math.toRadians(60.0)), 2000, new Rotation2d(Math.PI), 0.0, 0.0);

  public enum SuperState {
    DEFAULT,
    IDLE,
    SHOOT,
    INTAKING,
    SHOOTING,
    SHOOTING_NO_FEED,
    PASS,
    PASSING,
    ZERO,
    MANUAL_SHOOT,
    MANUAL_SHOOTING,
    PRESET_SHOOT,
    PRESET_SHOOTING,
    MANUAL_PASS, // TODO: implement ts and passing
    MANUAL_PASSING,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public Superstructure(
      SwerveSubsystem drive, Shooter shooter, Intake intake, Kicker kicker, Agitator agitator) {
    this.drive = drive;

    this.shooter = shooter;
    this.intake = intake;
    this.kicker = kicker;
    this.agitator = agitator;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
  }

  public void setWantedState(SuperState wantedState, ShotSolution shotSolution) {
    this.wantedSuperState = wantedState;
    this.presetShotSolution = shotSolution;
  }

  public Command setWantedSuperStateCommand(SuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  public SuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public SuperState getLastSuperState() {
    return lastState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case DEFAULT:
        break;
      case SHOOT:
        handleShootState();
        break;
      case SHOOTING:
        handleShootingState();
        break;

      case PASSING:
        handlePassingState();
        break;

      case INTAKING:
        handleIntakingState();
        break;
      case ZERO:
        break;

      default:
        handleIdleState();
        break;
    }
  }

  /**
   * This function handles the state transitions of the Superstructure subsystem. It updates the
   * current state based on the wanted state and performs necessary actions.
   *
   * @return SuperState - The current state of the Superstructure subsystem after handling the state
   *     transitions.
   * @param wantedSuperState The desired state of the Superstructure subsystem.
   * @see SuperState
   */
  private void handleIntakingState() {
    intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
  }

  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case DEFAULT:
        currentSuperState = SuperState.DEFAULT;
        break;
      case SHOOT:
        if (Constants.Field.isInAllianceZone(
            Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getTranslation())) {
          if (shooter.readyToShoot()
              && !Constants.Field.isOnBump(
                  Robotstate.getInstance()
                      .getRobotPoseFromSwerveDriveOdometry()
                      .getTranslation())) {
            currentSuperState = SuperState.SHOOTING;
          } else {
            currentSuperState = SuperState.SHOOT;
          }
          break;
        }
        if (shooter.readyToPass()) {
          currentSuperState = SuperState.PASSING;
        } else {
          currentSuperState = SuperState.PASS;
        }
        break;
      case PASS:
        if (shooter.readyToPass()) {
          currentSuperState = SuperState.PASSING;
        } else {
          currentSuperState = SuperState.PASS;
        }
        break;
      case MANUAL_SHOOT:
        currentSuperState = SuperState.MANUAL_SHOOTING;

        break;
      case MANUAL_SHOOTING:
        currentSuperState = SuperState.MANUAL_SHOOTING;
        break;
      case PRESET_SHOOT:
        if (shooter.readyToShoot()) {
          currentSuperState = SuperState.PRESET_SHOOTING;
        } else {
          currentSuperState = SuperState.PRESET_SHOOT;
        }
        break;
      case PRESET_SHOOTING:
        currentSuperState = SuperState.PRESET_SHOOTING;
        break;
      case INTAKING:
        currentSuperState = SuperState.INTAKING;
        break;
      case SHOOTING:
        currentSuperState = SuperState.SHOOTING;
        break;
      case SHOOTING_NO_FEED:
        currentSuperState = SuperState.SHOOTING_NO_FEED;
        break;
      case PASSING:
        currentSuperState = SuperState.PASSING;
        break;
      case ZERO:
        // if (intake.isZeroed()) {
        //   intake.setWantedState(IntakeState.DOWN);
        //   wantedSuperState = SuperState.DEFAULT;
        //   currentSuperState = SuperState.DEFAULT;
        // }
        currentSuperState = SuperState.ZERO;
        break;
    }
    return currentSuperState;
  }

  private void handleShootState() {
    // Shooter
    ShotSolution shotSolution =
        ShotCalculator.calculateHubShot(
            new Pose2d(
                getTurretFieldPosition().toTranslation2d(),
                Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation()),
            Constants.Field.getHubPose().toTranslation2d(),
            Robotstate.getInstance().getRobotChassisSpeeds());
    ShotSolution rotatedShotSolution =
        shotSolution.rotateTurretAngle(
            Robotstate.getInstance()
                .getRobotPoseFromSwerveDriveOdometry()
                .getRotation()
                .unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT, rotatedShotSolution);

    // Feeder
    kicker.setWantedKickerState(KickerWantedState.RUNNING);
    agitator.setWantedAgitatorState(WantedAgitatorState.SPINNING);
    intake.setWantedIntakeState(WantedIntakeState.PUMPING);
  }

  private void handleShootingState() {
    // Shooter
    ShotSolution shotSolution =
        ShotCalculator.calculateHubShot(
            new Pose2d(
                getTurretFieldPosition().toTranslation2d(),
                Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation()),
            Constants.Field.getHubPose().toTranslation2d(),
            Robotstate.getInstance().getRobotChassisSpeeds());
    ShotSolution rotatedShotSolution =
        shotSolution.rotateTurretAngle(
            Robotstate.getInstance()
                .getRobotPoseFromSwerveDriveOdometry()
                .getRotation()
                .unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT, rotatedShotSolution);
    // Feeder
    kicker.setWantedKickerState(KickerWantedState.RUNNING); // Pass ball into shooter
    agitator.setWantedAgitatorState(WantedAgitatorState.SPINNING);

    // Log Fuel Trajectory

    intake.setWantedIntakeState(WantedIntakeState.PUMPING);
  }

  private void handlePassingState() {
    // Shooter
    Translation3d turret = getTurretFieldPosition();
    ShotSolution shotSolution =
        ShotCalculator.calculateFeedShot(
            new Pose2d(
                turret.toTranslation2d(),
                Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation()),
            Constants.Field.getFeedTarget(turret.toTranslation2d()),
            Robotstate.getInstance().getRobotChassisSpeeds());
    ShotSolution rotatedShotSolution =
        shotSolution.rotateTurretAngle(
            Robotstate.getInstance()
                .getRobotPoseFromSwerveDriveOdometry()
                .getRotation()
                .unaryMinus());
    shooter.setWantedState(ShooterState.NORMAL_SHOOT, rotatedShotSolution);
    // Feeder
    kicker.setWantedKickerState(KickerWantedState.RUNNING); // Pass ball into shooter
    agitator.setWantedAgitatorState(WantedAgitatorState.SPINNING);

    // Log Fuel Trajectory

    intake.setWantedIntakeState(WantedIntakeState.PUMPING);
  }

  public Translation3d getTurretFieldPosition() {
    return new Translation3d(
            Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getX(),
            Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getY(),
            0.0)
        .plus(
            Constants.TurretConstants.TURRET_TRANSFORM_3D.rotateBy(
                new Rotation3d(
                    Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation())));
  }

  public void handleIdleState() {
    drive.setWantedState(WantedState.TELEOP_DRIVE);

    intake.setWantedIntakeState(WantedIntakeState.STOWED);
    agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
    kicker.setWantedKickerState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterState.IDLE);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Superstructure/turret field pose",
        new Pose3d(
            getTurretFieldPosition(),
            new Rotation3d(
                Robotstate.getInstance()
                    .getRobotPoseFromSwerveDriveOdometry()
                    .getRotation()
                    .plus(shooter.getRobotRelativeTurretAngle()))));

    Rotation2d turret =
        Constants.Field.getHubPose()
            .toTranslation2d()
            .minus(Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getTranslation())
            .getAngle();
    turret =
        turret.minus(Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation());
    shooter.passIdleTurretAngleToIdle(turret);

    currentSuperState = handleStateTransitions();
    if (RobotBase.isSimulation()) {
      for (int i = 0; i < trajectoryVelocity.size(); i++) {
        trajectoryVelocity.set(
            i,
            new Translation3d(
                trajectoryVelocity.get(i).getX(),
                trajectoryVelocity.get(i).getY(),
                trajectoryVelocity.get(i).getZ() - 9.81 * Globals.loopPeriodSecs));
        trajectoryPoint.set(
            i,
            trajectoryPoint.get(i).plus(trajectoryVelocity.get(i).times(Globals.loopPeriodSecs)));
        if (trajectoryPoint.get(i).getZ() < 0) {
          trajectoryPoint.remove(i);
          trajectoryVelocity.remove(i);
          i--;
        } else {
          Logger.recordOutput("Fuel/" + i, trajectoryPoint.get(i));
        }
      }
    }
    if (currentSuperState != tempLastState) {
      lastState = tempLastState;
      tempLastState = currentSuperState;
    }
    Logger.recordOutput("States/Super State", currentSuperState);
    Logger.recordOutput("Shooter/Manual Shoot RPM", manualShootRPM.get());
    Logger.recordOutput("Shooter/Manual Shoot Hood Angle", manualShootHoodAngle.get());
    Logger.recordOutput("Shooter/Manual Shoot Turret Angle", manualShootTurretAngle.get());
    Logger.recordOutput("Shooter/Ready to Shoot", shooter.readyToShoot());
    applyStates();
  }
}

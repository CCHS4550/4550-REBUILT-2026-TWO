package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIOCTRE;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIOCTRE;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterWantedState;
import frc.robot.Util.LaunchCalculator;

public class RobotContainer {
  private final Shooter shooter;
  private final LaunchCalculator calc = new LaunchCalculator();
  //   private final Intake intake;
  //   private final Turret turret;
  //   private final Vision vision;
  //   private final QuestNav questnav;

  //   private final Superstructure superstructure;
  //   private final SwerveSubsystem swerveSubsystem;
  //   private final Agitator agitator;
  //   private final Kicker kicker;
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();
    shooter = new Shooter(new ElevationIOCTRE(config), new FlywheelIOCTRE(config), calc);
    //     SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration,
    // CANcoderConfiguration>[]
    //         moduleConstants = config.getModuleConstants();

    //     swerveSubsystem =
    //         new SwerveSubsystem(
    //             new SwerveIOCTRE(config.getSwerveDrivetrainConstants(),
    // config.getModuleConstants()),
    //             config.geRobotConfig(),
    //             controller,
    //             moduleConstants[0].SpeedAt12Volts,
    //             moduleConstants[0].SpeedAt12Volts
    //                 / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
    //     // swerveSubsystem =
    //     //     new SwerveSubsystem(
    //     //         new SwerveIOCTRE(config.getSwerveDrivetrainConstants(),
    // config.getModuleConstants()),
    //     //         config.geRobotConfig(),
    //     //         controller,
    //     //         0.5,
    //     //         0.5 / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
    //     intake = new Intake(new IntakeIOCTRE(config));
    //     kicker = new Kicker(new KickerIOCTRE(config));
    //     agitator = new Agitator(new AgitatorIOCTRE(config));
    //     turret =
    //         new Turret(
    //             new ElevationIOCTRE(config), new RotationIOCTRE(config), new
    // ShooterIOCTRE(config));
    //     // turret = new Turret(new ElevationIOTest(), new RotationIOTest(), new ShooterIOTest());

    //     superstructure = new Superstructure(swerveSubsystem, intake, kicker, turret, agitator);
    //     questnav = new QuestNav(swerveSubsystem, new QuestNavIOQuest(Transform3d.kZero));
    //     vision =
    //         new Vision(
    //             questnav,
    //             new VisionIOPhotonvision("photonvision",
    // config.getVisionConfigurations().get(0)));

    //     // controller
    //     //     .a()
    //     //     .onTrue(
    //     //         new InstantCommand(
    //     //             () -> swerveSubsystem.setTargetRotation(Rotation2d.fromDegrees(90))));
    //     // controller
    //     //     .a()
    //     //     .whileTrue(
    //     //         new InstantCommand(() ->
    // swerveSubsystem.setWantedState(WantedState.ROTATION_LOCK)));
    //     // controller
    //     //     .a()
    //     //     .whileFalse(
    //     //         new InstantCommand(() ->
    // swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE)));

    //     // controller
    //     //     .b()
    //     //     .onTrue(
    //     //         new InstantCommand(
    //     //             () ->
    //     //                 swerveSubsystem.setDesiredPoseForDriveToPointWithConstraints(
    //     //                     new Pose2d(0.5, 0.5, new Rotation2d(Units.degreesToRadians(67))),
    //     //                     1,
    //     //                     3.14)));
    //     // controller
    //     //     .b()
    //     //     .whileTrue(
    //     //         new InstantCommand(() ->
    //     // swerveSubsystem.setWantedState(WantedState.DRIVE_TO_POINT)));

    //     // controller
    //     //     .b()
    //     //     .whileFalse(
    //     //         new InstantCommand(() ->
    // swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE)));
    //     // controller
    //     //     .x()
    //     //     .whileTrue(
    //     //         new InstantCommand(
    //     //             () -> intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE)));
    //     // controller
    //     //     .rightTrigger()
    //     //     .onTrue(
    //     //         new InstantCommand(
    //     //             () -> intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING)));
    //     // controller
    //     //     .rightTrigger()
    //     //     .onFalse(new InstantCommand(() ->
    // intake.setWantedIntakeState(WantedIntakeState.IDLE)));

    //     // controller
    //     //     .y()
    //     //     .onTrue(new InstantCommand(() ->
    // intake.setWantedIntakeState(WantedIntakeState.STOWED)));

    //     // controller
    //     //     .rightTrigger()
    //     //     .onTrue(
    //     //         new InstantCommand(
    //     //             () ->
    //     //                 turret.setWantedTurretMeasurables(
    //     //                     new TurretMeasurables(
    //     //                         new Rotation2d(Units.degreesToRadians(20)),
    //     //                         new Rotation2d(Units.degreesToRadians(20), 2)))));

    //     // controller
    //     //     .rightTrigger()
    //     //     .onTrue(
    //     //         new InstantCommand(
    //     //             () ->
    //     //                 turret.setWantedTurretMeasurables(
    //     //                     new TurretMeasurables(
    //     //                         new Rotation2d(0), new Rotation2d(45 * ((2 * Math.PI) /
    // 360))))));

    //     // controller
    //     //     .rightTrigger()
    //     //     .whileTrue(new InstantCommand(() ->
    // turret.setWantedState(TurretWantedState.TESTING)));

    //     // controller
    //     //     .rightTrigger()
    //     //     .onFalse(new InstantCommand(() -> turret.setWantedState(TurretWantedState.IDLE)));

    //     // controller
    //     //     .rightTrigger()
    //     //     .whileFalse(new InstantCommand(() ->
    // turret.setWantedState(TurretWantedState.IDLE)));

    //     // controller.start().onTrue(new InstantCommand(() ->
    // turret.setEncoderPositionAtBottom()));

    //     // controller
    //     //     .x()
    //     //     .whileFalse(
    //     //         new InstantCommand(
    //     //             () -> {
    //     //               kicker.setWantedState(Kicker.KickerWantedState.IDLE);
    //     //               agitator.setWantedAgitatorState(Agitator.WantedAgitatorState.IDLE);
    //     //               turret.setFlywheelSpeed(RadiansPerSecond.of(0.0));
    //     //               //   turret.setWantedState(Turret.TurretWantedState.IDLE);
    //     //             }));

    //     // actual button bindings!
    //     controller
    //         .a()
    //         .onTrue(
    //             new InstantCommand(
    //                 () -> swerveSubsystem.resetRotation(swerveSubsystem.getSwerveRotation())));

    //     // actual button bindings!
    //     // controller
    //     //     .a()
    //     //     .onTrue(new InstantCommand(() -> questnav.setPose(new Pose2d(3, 3, new
    // Rotation2d()))));

    //     // controller
    //     // .rightTrigger()
    //     // .whileTrue(new InstantCommand(() -> superstructure.setIntakeActive(true)))
    //     // .onFalse(new InstantCommand(() -> superstructure.setIntakeActive(false)));

    //     //     controller
    //     //         .rightBumper()
    //     //         .whileTrue(
    //     //             new InstantCommand(
    //     //                 () ->
    //     //                     superstructure.setWantedSuperstructureState(
    //     //                         WantedSuperstructureState.ACTIVE_SHOOT, true)))
    //     //         .whileFalse(
    //     //             new InstantCommand(
    //     //                 () ->
    //     //                     superstructure.setWantedSuperstructureState(
    //     //                         WantedSuperstructureState.IDLE, false)));
    //     //     controller
    //     //         .leftBumper()
    //     //         .whileTrue(
    //     //             new InstantCommand(
    //     //                 () ->
    //     //                     superstructure.setWantedSuperstructureState(
    //     //                         WantedSuperstructureState.STOW, false)))
    //     //         .whileFalse(
    //     //             new InstantCommand(
    //     //                 () ->
    //     //                     superstructure.setWantedSuperstructureState(
    //     //                         WantedSuperstructureState.IDLE, false)));

    //     controller
    //         .rightBumper()
    //         .whileTrue(new InstantCommand(() ->
    // turret.setWantedState(TurretWantedState.SHOOT_SCORE)))
    //         .whileFalse(new InstantCommand(() -> turret.setWantedState(TurretWantedState.IDLE)));
    controller
        .a()
        .whileTrue(
            new InstantCommand(() -> shooter.setWantedState(ShooterWantedState.FLYWHEEL_TEST)))
        .whileFalse(new InstantCommand(() -> shooter.setWantedState(ShooterWantedState.IDLE)));
  }

  //   public SwerveSubsystem getSwerveSubsystem() {
  //     return swerveSubsystem;
  //   }

  //   public void setTestTurretState() {
  //     turret.setWantedState(TurretWantedState.SHOOT_SCORE);
  //   }

  //   public void setTestPose() {
  //     swerveSubsystem.resetTranslationAndRotation(new Pose2d(3, 3, new Rotation2d()));
  //   }

  //   public boolean questPoseEstablished() {
  //     return questnav.questPoseEstablished();
  //   }

  //   public boolean isAtAutoStartingPose(Pose2d AutoStartingPose) {
  //     var distance =
  //         AutoStartingPose.getTranslation()
  //
  // .minus(Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getTranslation())
  //             .getNorm();
  //     return MathUtil.isNear(0.0, distance, 0.1);
  //   }

  //   public boolean isAtAutoStartingRotation(Rotation2d AutoStartingRotation) {
  //     return MathUtil.isNear(
  //
  // Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees(),
  //         AutoStartingRotation.getDegrees(),
  //         2);
  //   }
}

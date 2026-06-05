package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Subsystems.Drive.SwerveIOCTRE;
import frc.robot.Subsystems.Drive.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem;

  private final CommandXboxController controller = new CommandXboxController(0);

  // private double targetRPM;
  // private double kP;
  // private double kS;
  // private double kV;
  // private double flywheelVoltage;
  // private double wantedHoodAngle;

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();

    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
        moduleConstants = config.getModuleConstants();

    swerveSubsystem =
        new SwerveSubsystem(
            new SwerveIOCTRE(config.getSwerveDrivetrainConstants(), config.getModuleConstants()),
            config.geRobotConfig(),
            controller,
            moduleConstants[0].SpeedAt12Volts,
            moduleConstants[0].SpeedAt12Volts
                / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));

    // code to establish intaking
    // controller
    //     .leftTrigger()
    //     .and(controller.rightBumper().negate())
    //     .whileTrue(
    //         new InstantCommand(
    //             () ->
    //                 superstructure.setWantedSuperstructureState(
    //                     WantedSuperstructureState.INTAKING)))
    //     .onFalse(
    //         new InstantCommand(
    //             () ->
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // // code to establish outtaking
    // controller
    //     .b()
    //     .and(controller.rightBumper().negate())
    //     .whileTrue(
    //         new InstantCommand(
    //             () ->
    //                 superstructure.setWantedSuperstructureState(
    //                     WantedSuperstructureState.OUTTAKING)))
    //     .onFalse(
    //         new InstantCommand(
    //             () ->
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // // tare the intake if something goes wrong
    // controller.a().onTrue(new InstantCommand(() -> intake.tareTS()));

    // // code to establish stow
    // controller
    //     .x()
    //     .and(controller.rightBumper().negate())
    //     .whileTrue(
    //         new InstantCommand(
    //             () ->
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.STOW)))
    //     .onFalse(
    //         new InstantCommand(
    //             () ->
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // controller
    //     .y()
    //     .and(controller.rightBumper().negate())
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               superstructure.stopApplyingStates = true;
    //               intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
    //             }))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               superstructure.stopApplyingStates = false;
    //               intake.setWantedIntakeState(WantedIntakeState.IDLE);
    //             }));

    // // code for pre-aim, intaking (good for rev up)
    // controller
    //     .rightBumper()
    //     .and(controller.b())
    //     .and(controller.rightTrigger().negate())
    //     .whileTrue(
    //         new InstantCommand(
    //             () ->
    //                 superstructure.setWantedSuperstructureState(
    //                     WantedSuperstructureState.PRE_AIM_INTAKING)))
    //     .onFalse(
    //         new InstantCommand(
    //             () ->
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // // code for pre-aim, no intaking (more rev up opportunities)
    // controller
    //     .rightBumper()
    //     .and(controller.b().negate())
    //     .and(controller.rightTrigger().negate())
    //     .whileTrue(
    //         new InstantCommand(
    //             () ->
    //
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.PRE_AIM)))
    //     .onFalse(
    //         new InstantCommand(
    //             () ->
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // // code for shooting, everything should auto align and stuff
    // controller
    //     .rightBumper()
    //     .and(controller.rightTrigger())
    //     .and(controller.b().negate())
    //     .whileTrue(
    //         new InstantCommand(
    //             () ->
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.SHOOT)))
    //     .onFalse(
    //         new InstantCommand(
    //             () ->
    // superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // test code for manual shooting
    // This must be disabled before use of full bot
    // disable superstructure before use
    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   superstructure.stopApplyingStates = true;
    //                   shooter.setWantedState(ShooterWantedState.TEST);
    //                 }),
    //             new SequentialCommandGroup(
    //                 new WaitCommand(2),
    //                 new InstantCommand(
    //                     () -> {
    //                       indexer.setWantedState(IndexerWantedState.RUNNING);
    //                     }),
    //                 new InstantCommand(
    //                     () -> {
    //                       shooter.setWantedState(ShooterWantedState.TEST_2);
    //                     }),
    //                 new WaitCommand(0),
    //                 new InstantCommand(
    //                     () -> intake.setWantedIntakeState(WantedIntakeState.PUMPING)))))
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               superstructure.stopApplyingStates = false;
    //               shooter.setWantedState(ShooterWantedState.IDLE);
    //               indexer.setWantedState(IndexerWantedState.IDLE);
    //               intake.setWantedIntakeState(WantedIntakeState.IDLE);
    //             }));
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }

  // public void setTestPose() {
  //   swerveSubsystem.resetTranslationAndRotation(new Pose2d(3, 3, new Rotation2d()));
  // }

  public boolean isAtAutoStartingPose(Pose2d AutoStartingPose) {
    var distance =
        AutoStartingPose.getTranslation()
            .minus(Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getTranslation())
            .getNorm();
    return MathUtil.isNear(0.0, distance, 0.1);
  }

  public boolean isAtAutoStartingRotation(Rotation2d AutoStartingRotation) {
    return MathUtil.isNear(
        Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees(),
        AutoStartingRotation.getDegrees(),
        2);
  }
}

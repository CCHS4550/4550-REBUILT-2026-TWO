package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Subsystems.Drive.SwerveIOCTRE;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIOTest;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTest;
import frc.robot.Subsystems.QuestNav.QuestNav;
import frc.robot.Subsystems.QuestNav.QuestNavIOQuest;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIOTest;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIOTest;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.WantedSuperstructureState;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIOPhotonvision;

public class RobotContainer {
  private final Vision vision;
  private final QuestNav questnav;

  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;
  private final SwerveSubsystem swerveSubsystem;

  private final Superstructure superstructure;
  private final CommandXboxController controller = new CommandXboxController(0);

  // private double targetRPM;
  // private double kP;
  // private double kS;
  // private double kV;
  // private double flywheelVoltage;
  // private double wantedHoodAngle;

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();

    // intake = new Intake(new IntakeIOCTRE(config));
    // indexer = new Indexer(new IndexerIOCTRE(config));
    // shooter = new Shooter(new ElevationIOCTRE(config), new FlywheelIOCTRE(config), controller);

    intake = new Intake(new IntakeIOTest());
    indexer = new Indexer(new IndexerIOTest());
    shooter = new Shooter(new ElevationIOTest(), new FlywheelIOTest(), controller);

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

    questnav =
        new QuestNav(swerveSubsystem, new QuestNavIOQuest(config.getVisionConfigurations().get(1)));
    vision =
        new Vision(
            questnav,
            swerveSubsystem,
            new VisionIOPhotonvision("photonvision", config.getVisionConfigurations().get(0)));

    superstructure = new Superstructure(swerveSubsystem, intake, shooter, indexer);

    // code to establish intaking
    controller
        .b()
        .and(controller.rightBumper().negate())
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.INTAKING)))
        .onFalse(
            new InstantCommand(
                () -> superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // tare the intake if something goes wrong
    controller.a().onTrue(new InstantCommand(() -> intake.tareTS()));

    // code for pre-aim, intaking (good for rev up)
    controller
        .rightBumper()
        .and(controller.b())
        .and(controller.rightTrigger().negate())
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(
                        WantedSuperstructureState.PRE_AIM_INTAKING)))
        .onFalse(
            new InstantCommand(
                () -> superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // code for pre-aim, no intaking (more rev up opportunities)
    controller
        .rightBumper()
        .and(controller.b().negate())
        .and(controller.rightTrigger().negate())
        .whileTrue(
            new InstantCommand(
                () ->
                    superstructure.setWantedSuperstructureState(WantedSuperstructureState.PRE_AIM)))
        .onFalse(
            new InstantCommand(
                () -> superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // code for shooting, everything should auto align and stuff
    controller
        .rightBumper()
        .and(controller.rightTrigger())
        .and(controller.b().negate())
        .whileTrue(
            new InstantCommand(
                () -> superstructure.setWantedSuperstructureState(WantedSuperstructureState.SHOOT)))
        .onFalse(
            new InstantCommand(
                () -> superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    // test code for manual shooting
    // This must be disabled before use of full bot
    // disable superstructure before use
    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   shooter.setWantedState(ShooterWantedState.TEST);
    //                 }),
    //             new SequentialCommandGroup(
    //                 new WaitCommand(3),
    //                 new InstantCommand(
    //                     () -> {
    //                       indexer.setWantedState(IndexerWantedState.RUNNING);
    //                     }),
    //                 new InstantCommand(
    //                     () -> {
    //                       shooter.setWantedState(ShooterWantedState.TEST_2);
    //                     }),
    //                 new WaitCommand(0),
    //                 new InstantCommand(()->
    // intake.setWantedIntakeState(WantedIntakeState.PUMPING))
    //                     )))
    //     .whileFalse(
    //         new InstantCommand(
    //             () -> {
    //               shooter.setWantedState(ShooterWantedState.IDLE);
    //               indexer.setWantedState(IndexerWantedState.IDLE);
    //               intake.setWantedIntakeState(WantedIntakeState.IDLE);
    //             }));

  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  // public void setTestPose() {
  //   swerveSubsystem.resetTranslationAndRotation(new Pose2d(3, 3, new Rotation2d()));
  // }

  public boolean questPoseEstablished() {
    return questnav.questPoseEstablished();
  }

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

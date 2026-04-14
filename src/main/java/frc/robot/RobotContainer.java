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
import frc.robot.Subsystems.Indexer.IndexerIOCTRE;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.WantedIntakeState;
import frc.robot.Subsystems.Intake.IntakeIOCTRE;
import frc.robot.Subsystems.QuestNav.QuestNav;
import frc.robot.Subsystems.QuestNav.QuestNavIOQuest;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIOCTRE;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIOCTRE;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionIOPhotonvision;

public class RobotContainer {
  private final Vision vision;
  private final QuestNav questnav;

  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;
  private final SwerveSubsystem swerveSubsystem;
  private final CommandXboxController controller = new CommandXboxController(0);

  // private double targetRPM;
  // private double kP;
  // private double kS;
  // private double kV;
  // private double flywheelVoltage;
  private double wantedHoodAngle;

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();

    intake = new Intake(new IntakeIOCTRE(config));
    indexer = new Indexer(new IndexerIOCTRE(config));
    shooter = new Shooter(new ElevationIOCTRE(config), new FlywheelIOCTRE(config), controller);

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
            new VisionIOPhotonvision("photonvision", config.getVisionConfigurations().get(0)));

    // controller
    //     .a()
    //     .onTrue(new InstantCommand(() -> indexer.setWantedState(IndexerWantedState.RUNNING)))
    //     .onFalse(new InstantCommand(() -> indexer.setWantedState(IndexerWantedState.IDLE)));

    // controller
    //     .b()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
    //               System.out.println("Extending");
    //             }));
    // controller
    //     .b()
    //     .whileFalse(
    //         new InstantCommand(
    //             () -> {
    //               intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
    //               System.out.println("Intaking");
    //             }));

    // Change based on field testing
    // controller
    //     .rightBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               kV += changeMagnitude;
    //               System.out.println("flywheel voltage: " + kV);
    //             }));
    // controller
    //     .leftBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               kV -= changeMagnitude;
    //               System.out.println("flywheel voltage: " + kV);
    //             }));

    double flywheelSpeed = 200;
    // flywheelVoltage = 8;

    /*USE FOR FINDING INTERPOLATING STUFF */
    // elevation goes from 0 to 6 radians of motor rotation
    // goes from 20 to 60 degrees of mechanism rotation
    wantedHoodAngle = 20;

    double radiansFromDegrees = ((wantedHoodAngle - 20) / 40.0) * 6;
    // controller
    //     .rightTrigger()
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
    //                     }))))
    //     .whileFalse(
    //         new InstantCommand(
    //             () -> {
    //               shooter.setWantedState(ShooterWantedState.IDLE);
    //               indexer.setWantedState(IndexerWantedState.IDLE);
    //             }));
    controller
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.PUMPING)))
        .whileFalse(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.IDLE)));

    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               shooter.setFlywheelSpeed(shooterWrapper.flywheelVelo);
    //               shooter.setElevationAngle(shooterWrapper.hoodAngle);
    //             }))
    //     .whileFalse(
    //         new InstantCommand(
    //             () -> {
    //               shooter.setWantedState(ShooterWantedState.IDLE);
    //             }));

    // controller
    //     .rightTrigger()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               shooter.setWantedState(ShooterWantedState.TEST);
    //               ;
    //               System.out.println("Shooting!");
    //             }));
    // controller
    //     .rightTrigger()
    //     .onFalse(new InstantCommand(() -> shooter.setWantedState(ShooterWantedState.IDLE)));

    // controller
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING)));
    // controller
    //     .b()
    //     .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.STOWED)));
    // controller
    //     .x()
    //     .onTrue(new InstantCommand(() ->
    // intake.setWantedIntakeState(WantedIntakeState.PUMPING)));
    // controller
    //     .x()
    //     .onFalse(new InstantCommand(() ->
    // intake.setWantedIntakeState(WantedIntakeState.STOWED)));

    /* on second thought i don't like the way its written but ill keep it here for now ig
    /// for yall that are reading this
    /// "P" = kP
    /// "I" = kI
    /// and so on
    double changeMagnitude = 0.01;
    String slot = "P";
    int adjustedModule = 0; // 0 is flywheels, 1 is elevator, could add more later
    // These should print out the new slot value, if that doesn't happen, thats not good
    // btw ur welcom for this readable code, it should work
    controller
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  switch (adjustedModule) {
                    case (0):
                      shooter.adjustFlywheelKSlotValue(changeMagnitude, slot);
                      break;
                    case (1):
                      shooter.adjustElevationKSlotValue(changeMagnitude, slot);
                      break;
                    default:
                      System.out.println("Invalid module!!!!");
                      break;
                  }
                }));
    controller
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  switch (adjustedModule) {
                    case (0):
                      shooter.adjustFlywheelKSlotValue(-1 * changeMagnitude, slot);
                      break;
                    case (1):
                      shooter.adjustElevationKSlotValue(-1 * changeMagnitude, slot);
                      break;
                    default:
                      System.out.println("Invalid module!!!!");
                      break;
                  }
                }));
                */
  }

  // public SwerveSubsystem getSwerveSubsystem() {
  //   return swerveSubsystem;
  // }

  // public void setTestPose() {
  //   swerveSubsystem.resetTranslationAndRotation(new Pose2d(3, 3, new Rotation2d()));
  // }

  public boolean questPoseEstablished() {
    // return questnav.questPoseEstablished();
    return true;
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

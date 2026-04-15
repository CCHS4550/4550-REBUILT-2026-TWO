package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Drive.SwerveIOCTRE;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Drive.SwerveSubsystem.WantedState;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.Indexer.IndexerWantedState;
import frc.robot.Subsystems.Indexer.IndexerIOCTRE;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.WantedIntakeState;
import frc.robot.Subsystems.Intake.IntakeIOCTRE;
import frc.robot.Subsystems.QuestNav.QuestNav;
import frc.robot.Subsystems.QuestNav.QuestNavIOQuest;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIOCTRE;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIOCTRE;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterWantedState;
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
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Superstructure superstructure;

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

    superstructure = new Superstructure(swerveSubsystem, intake, shooter, indexer);


    controller
        .b()
        .whileTrue(
            new InstantCommand(
                () -> {
                  intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
                }));
    controller
        .b()
        .whileFalse(
            new InstantCommand(
                () -> {
                  intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
                  System.out.println("Intaking");
                }));
    
    controller
        .rightBumper()
        .whileTrue(
          new InstantCommand(
            ()-> 
              superstructure.setWantedSuperstructureState(WantedSuperstructureState.SHOOT)))
        .onFalse(
          new InstantCommand(()-> superstructure.setWantedSuperstructureState(WantedSuperstructureState.IDLE)));

    controller
        .leftTrigger()
        .whileTrue(
          new InstantCommand(()-> swerveSubsystem.setSwerveRequest(new SwerveRequest.SwerveDriveBrake()))
        )
        .onFalse(
          new InstantCommand(()-> swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE))
        );

    
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

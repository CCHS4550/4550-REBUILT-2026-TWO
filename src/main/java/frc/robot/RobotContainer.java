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
import frc.robot.Subsystems.Shooter.Elevation.ElevationIOCTRE;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIOCTRE;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.ShooterWantedState;

public class RobotContainer {
  // private final Vision vision;
  // private final QuestNav questnav;

  private final Shooter shooter;
  private final SwerveSubsystem swerveSubsystem;
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();

    shooter = new Shooter(new ElevationIOCTRE(config), new FlywheelIOCTRE(config));
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
    // questnav =
    //     new QuestNav(swerveSubsystem, new
    // QuestNavIOQuest(config.getVisionConfigurations().get(1)));
    // vision =
    //     new Vision(
    //         questnav,
    //         new VisionIOPhotonvision("photonvision", config.getVisionConfigurations().get(0)));

    controller
        .a()
        .onTrue(new InstantCommand(() -> shooter.setWantedState(ShooterWantedState.TEST)));
    controller
        .a()
        .onFalse(new InstantCommand(() -> shooter.setWantedState(ShooterWantedState.IDLE)));

    /// for yall retards that are reading this
    /// "P" = kP
    /// "I" = kI
    /// and so on
    double changeMagnitude = 0.01;
    String slot = "P";
    int adjustedModule = 0; //0 is flywheels, 1 or any other number ig is elevator
    // These should print out the new slot value, if that doesn't happen, thats not good
    // btw ur welcom for this readable code, it should work
    controller
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (adjustedModule == 0) shooter.adjustFlywheelKSlotValue(changeMagnitude, slot);
                  else shooter.adjustFlywheelKSlotValue(changeMagnitude, slot);
                }));
    controller
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (adjustedModule == 0) shooter.adjustFlywheelKSlotValue(changeMagnitude, slot);
                  else shooter.adjustFlywheelKSlotValue(-1 * changeMagnitude, slot);
                }));
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }

  public void setTestPose() {
    swerveSubsystem.resetTranslationAndRotation(new Pose2d(3, 3, new Rotation2d()));
  }

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

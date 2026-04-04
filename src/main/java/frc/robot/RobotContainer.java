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
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.WantedIntakeState;
import frc.robot.Subsystems.Intake.IntakeIOCTRE;

public class RobotContainer {

  private final Intake intake;

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    BruinRobotConfig config = new BruinRobotConfig();
    intake = new Intake(new IntakeIOCTRE(config));

    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
        moduleConstants = config.getModuleConstants();
    controller
        .a()
        .whileTrue(
            new InstantCommand(
                () -> intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE)));
    controller
        .a()
        .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.IDLE)));

    controller
        .b()
        .whileTrue(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.STOWED)));
    controller
        .b()
        .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.IDLE)));

    controller.x().onTrue(new InstantCommand(() -> intake.tare()));

    controller
        .y()
        .onTrue(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.PUMPING)));

    controller
        .y()
        .onFalse(new InstantCommand(() -> intake.setWantedIntakeState(WantedIntakeState.IDLE)));
  }

  // public SwerveSubsystem getSwerveSubsystem() {
  //   return swerveSubsystem;
  // }

  // public void setTestPose() {
  //   swerveSubsystem.resetTranslationAndRotation(new Pose2d(3, 3, new Rotation2d()));
  // }

  // public boolean questPoseEstablished() {
  //   return questnav.questPoseEstablished();
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

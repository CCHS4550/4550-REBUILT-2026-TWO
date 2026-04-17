package frc.robot.Subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final double intakeTolerance = 0.2;

  public enum WantedIntakeState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
    STOWED,
    PUMPING,
    BALL_STUFFING,
    IDLE
  }

  public enum SystemState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
    STOWED,
    STOW_SLOW,
    UPPER_PUMP,
    LOWER_PUMP,
    IDLE
  }

  private SystemState systemState = SystemState.EXTENDED_PASSIVE;
  private WantedIntakeState wantedState = WantedIntakeState.EXTENDED_PASSIVE;
  private final IntakeIO intakeIO;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  // So the big GPT said to do this, but I don't really know how it works and I hate converting
  // units so ts deprecated ig
  /*
    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> intakeIO.setExtensionVoltage(voltage.in(Volts)),
                log -> {
                  log.motor("extension")
                      .voltage(inputs.extensionIntakeVoltage.in(Volts))
                      .angularPosition(inputs.extensionPosRadians)
                      .angularVelocity(inputs.extensionIntakeVelocityRadPerSec);
                },
                this));

    public Command sysIdQuasistaticForward() {
      return sysIdRoutine
          .quasistatic(SysIdRoutine.Direction.kForward)
          .beforeStarting(() -> runningSysId = true)
          .finallyDo(() -> runningSysId = false);
    }
  */

  // public Command setExtensionToAngle(double angleRad) {
  //   return runOnce(() -> intakeIO.setExtensionMotorPositionRad(angleRad, 100, 50));
  // }

  /// private final CommandXboxController driverController = new CommandXboxController(0);
  /// driverController.a().onTrue(intake.setExtensionToAngle(1.0));

  public void tare() {
    intakeIO.tareExtensionPosition();
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDED_INTAKING:
        intakeIO.setSpinnerVoltage(5.0);
        intakeIO.setExtensionMotorPositionRad(
            Constants.IntakeConstants.INTAKE_BOTTOM_RADS, 100, 50);
        break;
      case EXTENDED_PASSIVE:
        intakeIO.setSpinnerVoltage(0.0);
        intakeIO.setExtensionMotorPositionRad(
            Constants.IntakeConstants.INTAKE_BOTTOM_RADS, 100, 50);
        break;
      case STOWED:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setExtensionMotorPositionRad(
            Constants.IntakeConstants.INTAKE_STOWED_RADS, 100, 50);
        break;
      case STOW_SLOW:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setExtensionMotorPositionRad(Constants.IntakeConstants.INTAKE_STOWED_RADS, 30, 25);
      case UPPER_PUMP:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setExtensionMotorPositionRad(
            Constants.IntakeConstants.INTAKE_TOP_PUMP_RADS, 30, 25);
        break;
      case LOWER_PUMP:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setExtensionMotorPositionRad(
            Constants.IntakeConstants.INTAKE_BOTTOM_PUMP_RADS, 30, 25);
        break;
      default:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setSpinnerVoltage(0.0);
        break;
    }
  }

  private SystemState handleStateTransitions() {
    switch (wantedState) {
      case EXTENDED_INTAKING:
        return SystemState.EXTENDED_INTAKING;
      case EXTENDED_PASSIVE:
        return SystemState.EXTENDED_PASSIVE;
      case STOWED:
        return SystemState.STOWED;
      case BALL_STUFFING:
        return SystemState.STOW_SLOW;
      case PUMPING:
        if (systemState == SystemState.UPPER_PUMP) {
          return atWantedAngle() ? SystemState.LOWER_PUMP : SystemState.UPPER_PUMP;
        } else if (systemState == SystemState.LOWER_PUMP) {
          return atWantedAngle() ? SystemState.UPPER_PUMP : SystemState.LOWER_PUMP;
        } else {
          return SystemState.LOWER_PUMP;
        }
        // return SystemState.LOWER_PUMP;
      case IDLE:
        return SystemState.IDLE;
      default:
        return SystemState.IDLE;
    }
  }

  public void setWantedIntakeState(WantedIntakeState state) {
    // FIX #2: Idempotency guard. Superstructure calls this every 20ms loop tick.
    // Without this, every call restarts the timer and re-evaluates location before
    // the mechanism has actually moved, immediately skipping timed transitions.
    if (state == this.wantedState) return;
    this.wantedState = state;
  }

  @AutoLogOutput(key = "Subsystems/Intake/AtWantedAngle")
  public boolean atWantedAngle() {
    switch (systemState) {
      case EXTENDED_INTAKING:
        return MathUtil.isNear(
            Constants.IntakeConstants.INTAKE_BOTTOM_RADS,
            inputs.extensionPosRadians,
            intakeTolerance);
      case EXTENDED_PASSIVE:
        return MathUtil.isNear(
            Constants.IntakeConstants.INTAKE_BOTTOM_RADS,
            inputs.extensionPosRadians,
            intakeTolerance);
      case STOWED:
        return MathUtil.isNear(
            Constants.IntakeConstants.INTAKE_BOTTOM_RADS,
            inputs.extensionPosRadians,
            intakeTolerance);
      case UPPER_PUMP:
        return MathUtil.isNear(
            Constants.IntakeConstants.INTAKE_TOP_PUMP_RADS,
            inputs.extensionPosRadians,
            intakeTolerance);
      case LOWER_PUMP:
        return MathUtil.isNear(
            Constants.IntakeConstants.INTAKE_BOTTOM_PUMP_RADS,
            inputs.extensionPosRadians,
            intakeTolerance);
      case IDLE:
        return true;
      default:
        return true;
    }
  }

  public void tareTS(){
    intakeIO.tareExtensionPosition();
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Subsystems/Intake", inputs);
    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/DesiredState", wantedState);
    Logger.recordOutput("Intake/AngleRad", inputs.extensionPosRadians);
    Logger.recordOutput("Intake/VelocityRadPerSec", inputs.extensionIntakeVelocityRadPerSec);
    Logger.recordOutput("Intake/AppliedVolts", inputs.extensionIntakeVoltage);
    systemState = handleStateTransitions();
    applyStates();
  }
}

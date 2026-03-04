package frc.robot.Subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// This code is Stupid and needs the be cleaned up.
public class Intake extends SubsystemBase {
  private final Timer timer = new Timer();

  public enum WantedIntakeState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
    STOWED,
    PUMPING,
    IDLE
  }

  public enum SystemState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
    MOVING_TO_EXTENSION_ACTIVE,
    MOVING_TO_EXTENSION_PASSIVE,
    STOWED,
    MOVING_TO_STOW,
    IDLE_AT_EXTENDED,
    IDLE_AT_STOW,
    MOVING_TO_UPPER_PUMP,
    MOVING_TO_LOWER_PUMP,
    IDLE_AT_LOWER_PUMP,
    IDLE_AT_UPPER_PUMP,
    UPPER_PUMP_TO_STOW,
    LOWER_PUMP_TO_STOW,
    UPPER_PUMP_TO_EXTENDED_ACTIVE,
    UPPER_PUMP_TO_EXTENDED_PASSIVE,
    LOWER_PUMP_TO_EXTENDED_ACTIVE,
    LOWER_PUMP_TO_EXTENDED_PASSIVE,
    EXTENDED_TO_LOWER_PUMP,
    STOW_TO_UPPER_PUMP
  }

  private enum Location {
    STOWED,
    UPPER_PUMP,
    LOWER_PUMP,
    EXTENDED
  }

  private Location location = Location.STOWED;
  private SystemState systemState = SystemState.STOWED;
  private WantedIntakeState wantedState = WantedIntakeState.STOWED;
  private final IntakeIO intakeIO;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDED_INTAKING:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setSpinnerVoltage(6.0);
        break;
      case MOVING_TO_EXTENSION_ACTIVE:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
        break;
      case MOVING_TO_EXTENSION_PASSIVE:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
        break;
      case MOVING_TO_STOW:
        intakeIO.setExtensionVoltage(3.0);
        intakeIO.setSpinnerVoltage(1.5);
        break;

      case MOVING_TO_UPPER_PUMP:
        intakeIO.setExtensionVoltage(3.0);
        intakeIO.setSpinnerVoltage(0);

      case MOVING_TO_LOWER_PUMP:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(0);

      case UPPER_PUMP_TO_STOW:
        intakeIO.setExtensionVoltage(3.0);
        intakeIO.setSpinnerVoltage(0.0);

      case LOWER_PUMP_TO_STOW:
        intakeIO.setExtensionVoltage(3.0);
        intakeIO.setSpinnerVoltage(1.5);

      case UPPER_PUMP_TO_EXTENDED_ACTIVE:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);

      case UPPER_PUMP_TO_EXTENDED_PASSIVE:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);

      case LOWER_PUMP_TO_EXTENDED_ACTIVE:
        intakeIO.setExtensionVoltage(-0.5);
        intakeIO.setSpinnerVoltage(0.0);

      case LOWER_PUMP_TO_EXTENDED_PASSIVE:
        intakeIO.setExtensionVoltage(-0.5);
        intakeIO.setSpinnerVoltage(0.0);

      case EXTENDED_TO_LOWER_PUMP:
        intakeIO.setExtensionVoltage(0.1);
        intakeIO.setSpinnerVoltage(0.0);

      case STOW_TO_UPPER_PUMP:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
      default:
        intakeIO.setExtensionVoltage(0);
        intakeIO.setSpinnerVoltage(0);
        break;
    }
  }

  public void setWantedIntakeState(WantedIntakeState state) {
    this.wantedState = state;
    switch (state) {
      case STOWED:
        if (location == Location.STOWED) {
          systemState = SystemState.STOWED;
        }
        if (location == Location.LOWER_PUMP) {
          systemState = SystemState.LOWER_PUMP_TO_STOW;
          timer.reset();
          timer.start();
        }
        if (location == Location.UPPER_PUMP) {
          systemState = SystemState.UPPER_PUMP_TO_STOW;
          timer.reset();
          timer.start();
        }
        if (location == Location.EXTENDED) {
          systemState = SystemState.MOVING_TO_STOW;
          timer.reset();
          timer.start();
        }
        location = Location.STOWED;
        break;
      case EXTENDED_INTAKING:
        if (location == Location.STOWED) {
          systemState = SystemState.MOVING_TO_EXTENSION_ACTIVE;
          timer.reset();
          timer.start();
        }
        if (location == Location.LOWER_PUMP) {
          systemState = SystemState.LOWER_PUMP_TO_EXTENDED_ACTIVE;
          timer.reset();
          timer.start();
        }
        if (location == Location.UPPER_PUMP) {
          systemState = SystemState.UPPER_PUMP_TO_EXTENDED_ACTIVE;
          timer.reset();
          timer.start();
        }
        if (location == Location.EXTENDED) {
          systemState = SystemState.EXTENDED_INTAKING;
        }
        location = Location.EXTENDED;
        break;
      case EXTENDED_PASSIVE:
        if (location == Location.STOWED) {
          systemState = SystemState.MOVING_TO_EXTENSION_PASSIVE;
          timer.reset();
          timer.start();
        }
        if (location == Location.LOWER_PUMP) {
          systemState = SystemState.LOWER_PUMP_TO_EXTENDED_PASSIVE;
          timer.reset();
          timer.start();
        }
        if (location == Location.UPPER_PUMP) {
          systemState = SystemState.UPPER_PUMP_TO_EXTENDED_PASSIVE;
          timer.reset();
          timer.start();
        }
        if (location == Location.EXTENDED) {
          systemState = SystemState.EXTENDED_PASSIVE;
        }
        location = Location.EXTENDED;
        break;
      case PUMPING:
        if (location == Location.EXTENDED) {
          systemState = SystemState.EXTENDED_TO_LOWER_PUMP;
          location = Location.LOWER_PUMP;
          timer.reset();
          timer.start();
          break;
        }

        if (location == Location.STOWED) {
          systemState = SystemState.STOW_TO_UPPER_PUMP;
          location = Location.UPPER_PUMP;
          timer.reset();
          timer.start();
          break;
        }

        if (location == Location.UPPER_PUMP) {
          systemState = SystemState.MOVING_TO_LOWER_PUMP;
          location = Location.LOWER_PUMP;
          timer.reset();
          timer.start();
          break;
        }

        if (location == Location.LOWER_PUMP) {
          systemState = SystemState.MOVING_TO_UPPER_PUMP;
          location = Location.UPPER_PUMP;
          timer.reset();
          timer.start();
          break;
        }
      case IDLE:
        if (location == Location.STOWED) {
          systemState = SystemState.IDLE_AT_STOW;
          timer.stop();
          timer.reset();
        }
        if (location == Location.LOWER_PUMP) {
          systemState = SystemState.IDLE_AT_LOWER_PUMP;
          timer.stop();
          timer.reset();
        }
        if (location == Location.UPPER_PUMP) {
          systemState = SystemState.IDLE_AT_UPPER_PUMP;
          timer.stop();
          timer.reset();
        }
        if (location == Location.EXTENDED) {
          systemState = SystemState.IDLE_AT_EXTENDED;
          timer.stop();
          timer.reset();
        }
        break;
      default:
        break;
    }
  }

  @AutoLogOutput(key = "Subsystems/Intake")
  public boolean atWantedAngle() {
    if (wantedState == WantedIntakeState.EXTENDED_INTAKING) {
      if (MathUtil.isNear(0.0, inputs.extensionPosRadians, 0.01)) {
        return true;
      }
    }
    if (wantedState == WantedIntakeState.STOWED) {
      if (MathUtil.isNear(Math.PI / 2, inputs.extensionPosRadians, 0.01)) {
        return true;
      }
    }
    return false;
  }
  ;

  // 0.25 works for pumping
  private void endTimerStateAndMoveOn() {
    if (systemState == SystemState.MOVING_TO_EXTENSION_PASSIVE && timer.hasElapsed(0.7)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.EXTENDED_PASSIVE;
      location = Location.EXTENDED;
    }
    if (systemState == SystemState.MOVING_TO_EXTENSION_ACTIVE && timer.hasElapsed(0.7)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.EXTENDED_INTAKING;
      location = Location.EXTENDED;
    }
    if (systemState == SystemState.MOVING_TO_STOW && timer.hasElapsed(0.75)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.STOWED;
      location = Location.STOWED;
    }

    if (systemState == SystemState.MOVING_TO_LOWER_PUMP && timer.hasElapsed(0.25)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.MOVING_TO_UPPER_PUMP;
      timer.start();
      location = Location.UPPER_PUMP;
    }
    if (systemState == SystemState.MOVING_TO_UPPER_PUMP && timer.hasElapsed(0.25)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.MOVING_TO_LOWER_PUMP;
      timer.start();
      location = Location.LOWER_PUMP;
    }

    if (systemState == SystemState.LOWER_PUMP_TO_EXTENDED_ACTIVE && timer.hasElapsed(0.05)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.EXTENDED_INTAKING;
      location = Location.EXTENDED;
    }

    if (systemState == SystemState.LOWER_PUMP_TO_EXTENDED_PASSIVE && timer.hasElapsed(0.05)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.EXTENDED_PASSIVE;
      location = Location.EXTENDED;
    }

    if (systemState == SystemState.UPPER_PUMP_TO_EXTENDED_ACTIVE && timer.hasElapsed(0.3)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.EXTENDED_INTAKING;
      location = Location.EXTENDED;
    }

    if (systemState == SystemState.UPPER_PUMP_TO_EXTENDED_PASSIVE && timer.hasElapsed(0.3)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.EXTENDED_PASSIVE;
      location = Location.EXTENDED;
    }

    if (systemState == SystemState.LOWER_PUMP_TO_STOW && timer.hasElapsed(0.7)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.STOWED;
      location = Location.STOWED;
    }

    if (systemState == SystemState.UPPER_PUMP_TO_STOW && timer.hasElapsed(0.4)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.STOWED;
      location = Location.STOWED;
    }

    if (systemState == SystemState.STOW_TO_UPPER_PUMP && timer.hasElapsed(0.4)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.MOVING_TO_LOWER_PUMP;
      timer.start();
      location = Location.LOWER_PUMP;
    }

    if (systemState == SystemState.EXTENDED_TO_LOWER_PUMP && timer.hasElapsed(0.0)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.MOVING_TO_UPPER_PUMP;
      timer.start();
      location = Location.UPPER_PUMP;
    }
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Subsystems/Intake", inputs);

    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/DesiredState", wantedState);
    endTimerStateAndMoveOn();
    applyStates();
  }
}

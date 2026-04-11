package frc.robot.Subsystems.Shooter.Flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double flywheel1AppliedVoltage = 0.0;
    public double flywheel1StatorCurrent = 0.0;
    public double flywheel1SupplyCurrent = 0.0;
    public double flywheel1Temperature = 0.0;

    public double flywheel2AppliedVoltage = 0.0;
    public double flywheel2StatorCurrent = 0.0;
    public double flywheel2SupplyCurrent = 0.0;
    public double flywheel2Temperature = 0.0;

    public double flywheel1VelocityRadPerSec = 0.0;
    public double flywheel1AccelRadPerSecPerSec = 0.0;

    public double flywheel2VelocityRadPerSec = 0.0;
    public double flywheel2AccelRadPerSecPerSec = 0.0;
  }

  default void updateInputs(FlywheelIOInputs io) {}

  default void setVoltage(Voltage voltage) {}

  default void setVelo(AngularVelocity velo) {}

  default void setSpeed(double rpm) {}

  default void adjustFlywheelKSlotValue(double value, String slot) {}
}

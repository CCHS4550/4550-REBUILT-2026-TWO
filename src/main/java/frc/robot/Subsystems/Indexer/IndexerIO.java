package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerIOInputs {
    public double indexerVoltage = 0.0;
    public double indexerSupplyCurrent = 0.0;
    public double indexerStatorCurrent = 0.0;
    public double indexerTemperature = 0.0;

    public double indexerVelocityRadPerSec = 0.0;
    public double indexerAccelRadPerSecSquared = 0.0;

    public double kickerVoltage = 0.0;
    public double kickerSupplyCurrent = 0.0;
    public double kickerStatorCurrent = 0.0;
    public double kickerTemperature = 0.0;

    public double kickerVelocityRadPerSec = 0.0;
    public double kickerAccelRadPerSecSquared = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setVoltage(double voltage) {}
}

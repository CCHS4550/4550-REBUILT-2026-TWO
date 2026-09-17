package frc.robot.Subsystems.Indexer;

import edu.wpi.first.units.measure.AngularVelocity;
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

    public double indexerTwoVoltage = 0.0;
    public double indexerTwoSupplyCurrent = 0.0;
    public double indexerTwoStatorCurrent = 0.0;
    public double indexerTwoTemperature = 0.0;

    public double indexerTwoVelocityRadPerSec = 0.0;
    public double indexerTwoAccelRadPerSecSquared = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setMotor1Voltage(double voltage) {}

  default void setMotor2Voltage(double voltage) {}

  default void setMotor1Velo(AngularVelocity velo) {}

  default void setMotor2Velo(AngularVelocity velo) {}
}

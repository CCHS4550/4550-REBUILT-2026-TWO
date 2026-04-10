package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Util.Phoenix6Util;

public class IndexerIOCTRE implements IndexerIO {
  private TalonFX indexerMotor;

  private TalonFX indexerTwoMotor;

  private TalonFXConfiguration indexerConfig;
  private TalonFXConfiguration indexerTwoConfig;

  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerSupplyCurrentAmps;
  private final StatusSignal<Current> indexerStatorCurrentAmps;
  private final StatusSignal<Temperature> indexerMotorTemp;
  private final StatusSignal<AngularVelocity> indexerVelocityRadPerSec;
  private final StatusSignal<AngularAcceleration> indexerAccelradPerSecSquared;

  private final StatusSignal<Voltage> indexerTwoAppliedVolts;
  private final StatusSignal<Current> indexerTwoSupplyCurrentAmps;
  private final StatusSignal<Current> indexerTwoStatorCurrentAmps;
  private final StatusSignal<Temperature> indexerTwoMotorTemp;
  private final StatusSignal<AngularVelocity> indexerTwoVelocityRadPerSec;
  private final StatusSignal<AngularAcceleration> indexerTwoAccelradPerSecSquared;

  public IndexerIOCTRE(BruinRobotConfig bruinRobotConfig) {
    indexerMotor =
        new TalonFX(
            bruinRobotConfig.INDEXER_MOTOR_1.getDeviceNumber(),
            bruinRobotConfig.INDEXER_MOTOR_1.getBus());

    indexerConfig = new TalonFXConfiguration();

    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    indexerConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(indexerMotor, indexerConfig, 5);

    indexerMotor.setPosition(0.0);
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerSupplyCurrentAmps = indexerMotor.getSupplyCurrent();
    indexerStatorCurrentAmps = indexerMotor.getStatorCurrent();
    indexerVelocityRadPerSec = indexerMotor.getVelocity();
    indexerAccelradPerSecSquared = indexerMotor.getAcceleration();
    indexerMotorTemp = indexerMotor.getDeviceTemp();

    indexerTwoMotor =
        new TalonFX(
            bruinRobotConfig.INDEXER_MOTOR_1.getDeviceNumber(),
            bruinRobotConfig.INDEXER_MOTOR_2.getBus());

    indexerTwoConfig = new TalonFXConfiguration();

    indexerTwoConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerTwoConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerTwoConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    indexerTwoConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    indexerTwoConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerTwoConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(indexerTwoMotor, indexerTwoConfig, 5);

    indexerTwoMotor.setPosition(0.0);
    indexerTwoAppliedVolts = indexerTwoMotor.getMotorVoltage();
    indexerTwoSupplyCurrentAmps = indexerTwoMotor.getSupplyCurrent();
    indexerTwoStatorCurrentAmps = indexerTwoMotor.getStatorCurrent();
    indexerTwoVelocityRadPerSec = indexerTwoMotor.getVelocity();
    indexerTwoAccelradPerSecSquared = indexerTwoMotor.getAcceleration();
    indexerTwoMotorTemp = indexerTwoMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        indexerAppliedVolts,
        indexerSupplyCurrentAmps,
        indexerStatorCurrentAmps,
        indexerVelocityRadPerSec,
        indexerAccelradPerSecSquared,
        indexerMotorTemp);

    BaseStatusSignal.refreshAll(
        indexerTwoAppliedVolts,
        indexerTwoSupplyCurrentAmps,
        indexerTwoStatorCurrentAmps,
        indexerTwoVelocityRadPerSec,
        indexerTwoAccelradPerSecSquared,
        indexerTwoMotorTemp);

    inputs.indexerVoltage = indexerAppliedVolts.getValueAsDouble();
    inputs.indexerSupplyCurrent = indexerSupplyCurrentAmps.getValueAsDouble();
    inputs.indexerStatorCurrent = indexerStatorCurrentAmps.getValueAsDouble();
    inputs.indexerVelocityRadPerSec = indexerVelocityRadPerSec.getValueAsDouble();
    inputs.indexerAccelRadPerSecSquared = indexerAccelradPerSecSquared.getValueAsDouble();
    inputs.indexerTemperature = indexerMotorTemp.getValueAsDouble();

    inputs.indexerTwoVoltage = indexerTwoAppliedVolts.getValueAsDouble();
    inputs.indexerTwoSupplyCurrent = indexerTwoSupplyCurrentAmps.getValueAsDouble();
    inputs.indexerTwoStatorCurrent = indexerTwoStatorCurrentAmps.getValueAsDouble();
    inputs.indexerTwoVelocityRadPerSec = indexerTwoVelocityRadPerSec.getValueAsDouble();
    inputs.indexerTwoAccelRadPerSecSquared =
        indexerTwoAccelradPerSecSquared.getValueAsDouble()
            * 2
            * Math.PI; // TODO: Update this later with a total gear reduction in constants
    inputs.indexerTwoTemperature = indexerTwoMotorTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    indexerMotor.setVoltage(voltage);
    indexerTwoMotor.setVoltage(voltage);
  }
}

package frc.robot.Subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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

  private MotionMagicVelocityVoltage indexerControl = new MotionMagicVelocityVoltage(0).withSlot(0);
  private MotionMagicVelocityVoltage indexerControl2 =
      new MotionMagicVelocityVoltage(0).withSlot(0);

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

    boolean currentLimit = true;

    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = currentLimit;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = currentLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    indexerConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    indexerConfig.Slot0.kP = 0.1;
    indexerConfig.Slot0.kI = 0.0;
    indexerConfig.Slot0.kD = 0.0;
    indexerConfig.Slot0.kV = 0.12;

    indexerConfig.MotionMagic.MotionMagicAcceleration = 1000;

    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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
            bruinRobotConfig.INDEXER_MOTOR_2.getDeviceNumber(),
            bruinRobotConfig.INDEXER_MOTOR_2.getBus());

    indexerTwoConfig = new TalonFXConfiguration();

    indexerTwoConfig.CurrentLimits.SupplyCurrentLimitEnable = currentLimit;
    indexerTwoConfig.CurrentLimits.StatorCurrentLimitEnable = currentLimit;
    indexerTwoConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
    indexerTwoConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    indexerTwoConfig.Slot0.kP = 0.1;
    indexerTwoConfig.Slot0.kI = 0.0;
    indexerTwoConfig.Slot0.kD = 0.0;
    indexerTwoConfig.Slot0.kV = 0.12;

    indexerTwoConfig.MotionMagic.MotionMagicAcceleration = 1000;

    indexerTwoConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerTwoConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
    indexerTwoMotor.setVoltage(voltage * 0.5);
  }

  @Override
  public void setMotor1Voltage(double voltage) {
    indexerMotor.setVoltage(voltage);
  }

  @Override
  public void setMotor2Voltage(double voltage) {
    indexerTwoMotor.setVoltage(voltage);
  }

  @Override
  public void setMotor1Velo(AngularVelocity velo) {
    indexerMotor.setControl(indexerControl.withVelocity(velo));
  }

  @Override
  public void setMotor2Velo(AngularVelocity velo) {
    indexerTwoMotor.setControl(indexerControl2.withVelocity(velo));
  }
}

package frc.robot.Subsystems.Shooter.Flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Util.Phoenix6Util;

public class FlywheelIOCTRE implements FlywheelIO {
  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;

  private MotionMagicVelocityVoltage flywheelControl =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private MotionMagicVelocityVoltage flywheelControl2 =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  private TalonFXConfiguration shooterConfig;

  private final StatusSignal<Voltage> flywheel1AppliedVoltage;
  private final StatusSignal<Current> flywheel1SupplyCurrent;
  private final StatusSignal<Current> flywheel1StatorCurrent;
  private final StatusSignal<Temperature> flywheel1Temperature;

  private final StatusSignal<AngularVelocity> flywheel1VelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> flywheel1AccelerationRotationsPerSecSquared;

  private final StatusSignal<Voltage> flywheel2AppliedVoltage;
  private final StatusSignal<Current> flywheel2SupplyCurrent;
  private final StatusSignal<Current> flywheel2StatorCurrent;
  private final StatusSignal<Temperature> flywheel2Temperature;

  private final StatusSignal<AngularVelocity> flywheel2VelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> flywheel2AccelerationRotationsPerSecSquared;

  public FlywheelIOCTRE(BruinRobotConfig config) {
    flywheelMotor1 =
        new TalonFX(config.FLYWHEEL_MOTOR_1.getDeviceNumber(), config.FLYWHEEL_MOTOR_1.getBus());
    flywheelMotor2 =
        new TalonFX(config.FLYWHEEL_MOTOR_2.getDeviceNumber(), config.FLYWHEEL_MOTOR_2.getBus());

    shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    shooterConfig.Slot0.kP = config.getShooterConfig().shooterKp;
    shooterConfig.Slot0.kI = config.getShooterConfig().shooterKi;
    shooterConfig.Slot0.kD = config.getShooterConfig().shooterKd;
    shooterConfig.Slot0.kS = config.getShooterConfig().shooterKs;
    shooterConfig.Slot0.kV = config.getShooterConfig().shooterKv;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterConfig.MotionMagic.MotionMagicAcceleration = 10000;

    // change this later!

    Phoenix6Util.applyAndCheckConfiguration(flywheelMotor1, shooterConfig, 5);
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    Phoenix6Util.applyAndCheckConfiguration(flywheelMotor2, shooterConfig, 5);

    flywheel1AppliedVoltage = flywheelMotor1.getMotorVoltage();
    flywheel1StatorCurrent = flywheelMotor1.getStatorCurrent();
    flywheel1SupplyCurrent = flywheelMotor1.getSupplyCurrent();
    flywheel1Temperature = flywheelMotor1.getDeviceTemp();

    flywheel1VelocityRotationsPerSec = flywheelMotor1.getVelocity();
    flywheel1AccelerationRotationsPerSecSquared = flywheelMotor1.getAcceleration();

    flywheel2AppliedVoltage = flywheelMotor2.getMotorVoltage();
    flywheel2StatorCurrent = flywheelMotor2.getStatorCurrent();
    flywheel2SupplyCurrent = flywheelMotor2.getSupplyCurrent();
    flywheel2Temperature = flywheelMotor2.getDeviceTemp();

    flywheel2VelocityRotationsPerSec = flywheelMotor2.getVelocity();
    flywheel2AccelerationRotationsPerSecSquared = flywheelMotor2.getAcceleration();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        flywheel1AppliedVoltage,
        flywheel1StatorCurrent,
        flywheel1SupplyCurrent,
        flywheel1Temperature,
        flywheel1VelocityRotationsPerSec,
        flywheel1AccelerationRotationsPerSecSquared);

    BaseStatusSignal.refreshAll(
        flywheel2AppliedVoltage,
        flywheel2StatorCurrent,
        flywheel2SupplyCurrent,
        flywheel2Temperature,
        flywheel2VelocityRotationsPerSec,
        flywheel2AccelerationRotationsPerSecSquared);

    inputs.flywheel1AppliedVoltage = flywheel1AppliedVoltage.getValueAsDouble();
    inputs.flywheel1StatorCurrent = flywheel1StatorCurrent.getValueAsDouble();
    inputs.flywheel1SupplyCurrent = flywheel1SupplyCurrent.getValueAsDouble();
    inputs.flywheel1Temperature = flywheel1Temperature.getValueAsDouble();

    inputs.flywheel1VelocityRadPerSec =
        Units.rotationsToRadians(flywheel1VelocityRotationsPerSec.getValueAsDouble())
            * Constants.ShooterConstants.SHOOTER_ONE_GEAR_RATIO;
    inputs.flywheel1AccelRadPerSecPerSec =
        Units.rotationsToRadians(flywheel1AccelerationRotationsPerSecSquared.getValueAsDouble())
            * Constants.ShooterConstants.SHOOTER_ONE_GEAR_RATIO;

    inputs.flywheel2AppliedVoltage = flywheel2AppliedVoltage.getValueAsDouble();
    inputs.flywheel2StatorCurrent = flywheel2StatorCurrent.getValueAsDouble();
    inputs.flywheel2SupplyCurrent = flywheel2SupplyCurrent.getValueAsDouble();
    inputs.flywheel2Temperature = flywheel2Temperature.getValueAsDouble();

    inputs.flywheel2VelocityRadPerSec =
        Units.rotationsToRadians(flywheel2VelocityRotationsPerSec.getValueAsDouble())
            * Constants.ShooterConstants.SHOOTER_TWO_GEAR_RATIO;
    inputs.flywheel2AccelRadPerSecPerSec =
        Units.rotationsToRadians(
            flywheel2AccelerationRotationsPerSecSquared.getValueAsDouble()
                * Constants.ShooterConstants.SHOOTER_TWO_GEAR_RATIO);
  }

  @Override
  public void setVoltage(
      Voltage voltage) { // why is this voltage and not just an int, what other unit would this be?
    // TODO: fix
    flywheelMotor1.setVoltage(voltage.magnitude());
    flywheelMotor2.setVoltage(voltage.magnitude());
  }

  @Override
  public void setVelo(AngularVelocity velo) {
    flywheelMotor1.setControl(
        flywheelControl
            .withVelocity(
                velo.in(RotationsPerSecond) / Constants.ShooterConstants.SHOOTER_ONE_GEAR_RATIO)
            .withSlot(0)
            .withEnableFOC(true));
    flywheelMotor2.setControl(
        flywheelControl2
            .withVelocity(
                velo.in(RotationsPerSecond) / Constants.ShooterConstants.SHOOTER_TWO_GEAR_RATIO)
            .withSlot(0)
            .withEnableFOC(true));
  }

  @Override
  public void setSpeed(double rpm) {
    double feedforward = (shooterConfig.Slot0.kV * rpm) + shooterConfig.Slot0.kS;
    double error = rpm - flywheel1VelocityRotationsPerSec.getValueAsDouble() * 2 * Math.PI;
    double feedback = error * shooterConfig.Slot0.kP;

    setVoltage(Voltage.ofBaseUnits(feedback + feedforward, Volts));
    System.out.println("Error" + error);
    System.out.println("Target: " + rpm);
    System.out.println("Velo: " + (flywheelMotor1.getVelocity().getValueAsDouble() * 2 * Math.PI));
  }

  @Override
  public void adjustFlywheelKSlotValue(double value, String slot) {
    System.out.print("Flywheel k" + slot + " value: ");
    switch (slot) {
      case "P":
        shooterConfig.Slot0.kP += value;
        System.out.println(shooterConfig.Slot0.kP);
        break;

      case "I":
        shooterConfig.Slot0.kI += value;
        System.out.println(shooterConfig.Slot0.kI);
        break;

      case "D":
        shooterConfig.Slot0.kD += value;
        System.out.println(shooterConfig.Slot0.kD);
        break;

      case "S":
        shooterConfig.Slot0.kS += value;
        System.out.println(shooterConfig.Slot0.kS);
        break;

      case "V":
        shooterConfig.Slot0.kV += value;
        System.out.println(shooterConfig.Slot0.kV);
        break;

      default:
        System.out.println("Invalid slot!!!!!");
        break;
    }

    Phoenix6Util.applyAndCheckConfiguration(flywheelMotor1, shooterConfig, 5);
    Phoenix6Util.applyAndCheckConfiguration(flywheelMotor2, shooterConfig, 5);
  }
}

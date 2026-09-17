package frc.robot.Subsystems.Shooter.Elevation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Util.Phoenix6Util;

public class ElevationIOCTRE implements ElevationIO {
  private TalonFX elevationMotor;
  private TalonFXConfiguration elevationConfig;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private final StatusSignal<Angle> elevationAngleRotations;
  private final StatusSignal<Voltage> elevationAppliedVolts;
  private final StatusSignal<Current> elevationSupplyCurrentAmps;
  private final StatusSignal<Current> elevationStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> elevationVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> elevationAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> elevationMotorTemp;

  public ElevationIOCTRE(BruinRobotConfig bruinRobotConfig) {
    elevationMotor =
        new TalonFX(
            bruinRobotConfig.ELEVATION_MOTOR.getDeviceNumber(),
            bruinRobotConfig.ELEVATION_MOTOR.getBus()); // creates motor

    // I should probably set up these constants in like RobotConfig, but I just want to try and
    // complete this out

    elevationConfig = new TalonFXConfiguration();
    elevationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevationConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevationConfig.CurrentLimits.SupplyCurrentLimit = 30;
    elevationConfig.CurrentLimits.StatorCurrentLimit = 40.0;

    elevationConfig.Slot0.kP = bruinRobotConfig.getShooterConfig().elevationKp;
    elevationConfig.Slot0.kI = bruinRobotConfig.getShooterConfig().elevationKi;
    elevationConfig.Slot0.kD = bruinRobotConfig.getShooterConfig().elevationKd;
    elevationConfig.Slot0.kS = bruinRobotConfig.getShooterConfig().elevationKs;
    elevationConfig.Slot0.kV = bruinRobotConfig.getShooterConfig().elevationKv;
    elevationConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevationConfig.Slot0.kG = -0.3; // change
    elevationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevationConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevationConfig.MotionMagic.MotionMagicCruiseVelocity = 64.4;
    elevationConfig.MotionMagic.MotionMagicAcceleration = 75.3; // some constant idk

    Phoenix6Util.applyAndCheckConfiguration(elevationMotor, elevationConfig, 5);

    elevationMotor.setPosition(
        Constants.ShooterConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS
            / Constants.ShooterConstants.ELEVATION_POSITION_COEFFICIENT);
    elevationAngleRotations = elevationMotor.getPosition();
    elevationAppliedVolts = elevationMotor.getMotorVoltage();
    elevationSupplyCurrentAmps = elevationMotor.getSupplyCurrent();
    elevationStatorCurrentAmps = elevationMotor.getStatorCurrent();
    elevationVelocityRotationsPerSec = elevationMotor.getVelocity();
    elevationAccelerationRotationsPerSecSquared = elevationMotor.getAcceleration();
    elevationMotorTemp = elevationMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(ElevationIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevationAppliedVolts,
        elevationSupplyCurrentAmps,
        elevationStatorCurrentAmps,
        elevationMotorTemp,
        elevationAccelerationRotationsPerSecSquared);
    BaseStatusSignal.refreshAll(elevationAngleRotations, elevationVelocityRotationsPerSec);
    inputs.elevationVoltage = elevationAppliedVolts.getValueAsDouble();
    inputs.elevationSupplyCurrent = elevationSupplyCurrentAmps.getValueAsDouble();
    inputs.elevationStatorCurrent = elevationStatorCurrentAmps.getValueAsDouble();
    inputs.elevationTemperature = elevationMotorTemp.getValueAsDouble();

    inputs.elevationVelocityRadPerSec =
        elevationVelocityRotationsPerSec.getValueAsDouble()
            * Constants.ShooterConstants.ELEVATION_POSITION_COEFFICIENT;
    inputs.elevationAccelRadPerSecSquared =
        elevationAccelerationRotationsPerSecSquared.getValueAsDouble()
            * Constants.ShooterConstants.ELEVATION_POSITION_COEFFICIENT;

    inputs.elevationAngle =
        Rotation2d.fromRadians(
            elevationAngleRotations.getValueAsDouble()
                * Constants.ShooterConstants.ELEVATION_POSITION_COEFFICIENT);
    // Rotation2d.fromRadians(
    //     mapRange(
    //         (Math.PI)
    //             - (elevationAngleRotations.getValueAsDouble()
    //                 * Constants.ShooterConstants.ELEVATION_ENCODER_POSITION_COEFFICIENT)
    //             - Units.degreesToRadians(77.312)));
  }

  @Override
  public void setElevationAngle(Rotation2d angle) {
    angle =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(),
                Constants.ShooterConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS,
                Constants.ShooterConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS));

    double wantedAngle =
        angle.getRadians() / Constants.ShooterConstants.ELEVATION_POSITION_COEFFICIENT;

    elevationMotor.setControl(motionMagicVoltage.withPosition(wantedAngle));

    // angle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), 0.02, 4.3)); // fill

    // double wantedAngle = angle.getRadians();
    // double error = wantedAngle - elevationMotor.getPosition().getValueAsDouble();
    // double kP = 1.5;
    // double kS = 0.41;
    // setVoltage(error * kP + kS);

    // System.out.println(wantedAngle);
    // System.out.println(error);
    // System.out.println("Voltage: " + (error * kP + kS));
  }

  @Override
  public void setVoltage(double volts) {
    elevationMotor.setVoltage(volts);
  }

  @Override
  public void adjustElevationKSlotValue(double value, String slot) {
    System.out.print("Elevation k" + slot + " value: ");
    switch (slot) {
      case "P":
        elevationConfig.Slot0.kP += value;
        System.out.println(elevationConfig.Slot0.kP);
        break;

      case "I":
        elevationConfig.Slot0.kI += value;
        System.out.println(elevationConfig.Slot0.kI);
        break;

      case "D":
        elevationConfig.Slot0.kD += value;
        System.out.println(elevationConfig.Slot0.kD);
        break;

      case "S":
        elevationConfig.Slot0.kS += value;
        System.out.println(elevationConfig.Slot0.kS);
        break;

      case "V":
        elevationConfig.Slot0.kV += value;
        System.out.println(elevationConfig.Slot0.kV);
        break;

      default:
        System.out.println("Invalid slot!!!!!");
        break;
    }

    Phoenix6Util.applyAndCheckConfiguration(elevationMotor, elevationConfig, 5);
  }
}

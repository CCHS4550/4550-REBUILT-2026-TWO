package frc.robot.Subsystems.Shooter.Elevation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Util.Phoenix6Util;

public class ElevationIOSim implements ElevationIO {
  private DCMotor elevationMotorModel = DCMotor.getKrakenX44Foc(1);
  
  private SingleJointedArmSim sim = 
    new SingleJointedArmSim(
          elevationMotorModel,
          1.0,
          .004,
          .33,
          Units.degreesToRadians(5),
          Units.degreesToRadians(45),
          false,
          Units.degreesToRadians(5));

  
  private TalonFXConfiguration elevationConfig;
  private CANcoderConfiguration encoderConfig;
  private PIDController controller = new PIDController (1,0,0);
  

  
  public ElevationIOSim() {}

  @Override
  public void updateInputs(ElevationIOInputs inputs) {
   
    sim.update(Constants.loopPeriodSecs);

    inputs.elevationVoltage = elevationMotorModel.getVoltage(0.198 * sim.getCurrentDrawAmps(), sim.getVelocityRadPerSec());
    inputs.elevationStatorCurrent = sim.getCurrentDrawAmps();
    inputs.elevationSupplyCurrent = sim.getCurrentDrawAmps();
    inputs.elevationTemperature = 0.0;

    inputs.elevationAngle = Rotation2d.fromRadians(sim.getAngleRads());
    inputs.elevationVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.elevationAccelRadPerSecSquared = sim.getVelocityRadPerSec(); // no function for acceleration
  }

  @Override
  public void setElevationAngle(Rotation2d angle) {

    if (angle.getRadians() > Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS) {
      angle =
          Rotation2d.fromRadians(
              Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);
    }
    if (angle.getRadians()
        < Constants.TurretConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS) {
      angle =
          Rotation2d.fromRadians(
              Constants.TurretConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);
    }

   sim.setInputVoltage(controller.calculate(angle.getRadians()));
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  public static double mapRange(double value) {
    // Check to prevent division by zero if the old range is invalid

    // Perform the linear mapping
    double oldRange = 2.3643 - 1.3024;
    double newRange = 1.3788 - 0.7505;
    return 0.7505 + ((value - 1.3024) * newRange / oldRange);
  }
}

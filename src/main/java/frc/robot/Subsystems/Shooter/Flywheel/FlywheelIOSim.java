package frc.robot.Subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Subsystems.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;
import frc.robot.Util.Phoenix6Util;

public class FlywheelIOSim implements FlywheelIO {
  private DCMotor motorType = DCMotor.getKrakenX60Foc(3);
  private DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorType, .3, 1), motorType);


  private MotionMagicVelocityVoltage flywheelControl;

  private PIDController controller = new PIDController(1, 0, 0);

  private double currentOutput = 0.0;
private double currentOutputAsVolt = 0.0;
  private double appliedVolts = 0.0;
  

  public FlywheelIOSim(BruinRobotConfig config) {

    sim.update(Constants.loopPeriodSecs);
    
    

  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

     currentOutputAsVolt =
        MathUtil.clamp(
            motorType.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
    appliedVolts = currentOutputAsVolt;

    inputs.flywheel1AppliedVoltage = sim.getInputVoltage();
    inputs.flywheel1StatorCurrent = sim.getCurrentDrawAmps();
    inputs.flywheel1SupplyCurrent = sim.getCurrentDrawAmps();
    inputs.flywheel1Temperature = 0.0;


    inputs.flywheel2AppliedVoltage = sim.getInputVoltage();
    inputs.flywheel2StatorCurrent = sim.getCurrentDrawAmps();
    inputs.flywheel2SupplyCurrent = sim.getCurrentDrawAmps();
    inputs.flywheel2Temperature = 0.0;

    inputs.flywheel3AppliedVoltage = sim.getInputVoltage();
    inputs.flywheel3StatorCurrent = sim.getCurrentDrawAmps();
    inputs.flywheel3SupplyCurrent = sim.getCurrentDrawAmps();
    inputs.flywheel3Temperature = 0.0;
    
  }

  @Override
  public void setVoltage(Voltage voltage) {
    sim.setInputVoltage(voltage.magnitude());
  }

  @Override
  public void setVelo(AngularVelocity velo) {
    sim.setInputVoltage(controller.calculate(velo.magnitude()));
  }
}

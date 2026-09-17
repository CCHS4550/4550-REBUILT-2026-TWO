package frc.robot.Subsystems.Shooter.Flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOTest implements FlywheelIO {

  public FlywheelIOTest() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {}

  @Override
  public void setVoltage(
      Voltage voltage) { // why is this voltage and not just an int, what other unit would this be?
    // TODO: fix

  }

  @Override
  public void setVelo(AngularVelocity velo) {}

  @Override
  public void setSpeed(double rpm) {}

  @Override
  public void adjustFlywheelKSlotValue(double value, String slot) {}
}

package frc.robot.Subsystems.Intake;

import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeIOTest implements IntakeIO {

  public IntakeIOTest() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void setExtensionMotorPositionRad(
      double rad, double veloRotPerSec, double accelRotPerSec) {}

  @Override
  public void setExtensionVoltage(double voltage) {}

  @Override
  public void setSpinnerVoltage(double voltage) {}

  @Override
  public void setSpinnerVelo(AngularVelocity velo) {}

  @Override
  public void tareExtensionPosition() {}
}

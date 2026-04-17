package frc.robot.Subsystems.Shooter.Elevation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Shooter.Elevation.ElevationIO.ElevationIOInputs;

public class ElevationIOTest implements ElevationIO {

  public ElevationIOTest() {}

  @Override
  public void updateInputs(ElevationIOInputs inputs) {}

  @Override
  public void setElevationAngle(Rotation2d angle) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void adjustElevationKSlotValue(double value, String slot) {}
}

package frc.robot.subsystems.coral_intake.pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOSim implements PivotIO {

  @Override
  public void setPosition(Angle position) {
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
  }

  @Override
  public void setVoltage(Voltage voltage) {
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }
}

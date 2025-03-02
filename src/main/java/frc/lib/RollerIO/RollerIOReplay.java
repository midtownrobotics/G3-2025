package frc.lib.RollerIO;

import edu.wpi.first.units.measure.Voltage;

public class RollerIOReplay implements RollerIO {

  @Override
  public void setVoltage(Voltage voltage) {
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public void setOutput(double output) {

  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }
}

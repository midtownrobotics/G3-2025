package frc.lib.RollerIO;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class RollerIOSim implements RollerIO {
  private Voltage voltage = Volts.zero();

  @Override
  public void setVoltage(Voltage voltage) {
    this.voltage = voltage;
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    inputs.appliedVoltage = voltage;
  }
}

package frc.robot.subsystems.coral_intake.belt;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class BeltIOSim implements BeltIO {
  private Voltage voltage = Volts.of(0);

  @Override
  public void setVoltage(Voltage voltage) {
    this.voltage = voltage;
  }

  @Override
  public void updateInputs(BeltInputs inputs) {
    inputs.appliedVoltage = voltage;
  }
}

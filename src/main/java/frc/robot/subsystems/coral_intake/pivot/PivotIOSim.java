package frc.robot.subsystems.coral_intake.pivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOSim implements PivotIO {
  private Angle targetPosition = Degrees.of(0);

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.position = targetPosition;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }
}

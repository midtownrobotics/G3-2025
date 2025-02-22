package frc.robot.subsystems.coral_intake.pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class PivotIOReplay implements PivotIO {

  @Override
  public void setPositionWithFeedforward(Angle position, Angle a, Voltage ff) {
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void setVoltage(Voltage voltage) {
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }
}

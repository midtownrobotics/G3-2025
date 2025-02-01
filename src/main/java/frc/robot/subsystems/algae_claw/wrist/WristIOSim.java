package frc.robot.subsystems.algae_claw.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class WristIOSim implements WristIO {

  @Override
  public void setPosition(Angle position) {
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public DutyCycleEncoder getEncoder() {
    throw new UnsupportedOperationException("Unimplemented method 'getEncoder'");
  }
}

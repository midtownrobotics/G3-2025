package frc.robot.subsystems.elevator.winch;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class WinchIOSim implements WinchIO {
  private Distance height = Feet.zero();

  @Override
  public void updateInputs(WinchInputs inputs) {
    inputs.left.position = height;
    inputs.right.position = height;
  }

  @Override
  public void setScorePosition(Distance position) {
    height = position;
  }

  @Override
  public void setClimbPosition(Distance position) {
    height = position;
  }

  @Override
  public Distance getPosition() {
    return Inches.zero();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }
}

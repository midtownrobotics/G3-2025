package frc.robot.subsystems.elevator.winch;

import edu.wpi.first.units.measure.Distance;

public class WinchIOSim implements WinchIO {
  @Override
  public void updateInputs(WinchInputs inputs) {
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void setScorePosition(Distance position) {
    throw new UnsupportedOperationException("Unimplemented method 'setScorePosition'");
  }

  @Override
  public void setClimbPosition(Distance position) {
    throw new UnsupportedOperationException("Unimplemented method 'setClimbPosition'");
  }

  @Override
  public Distance getPosition() {
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }
}

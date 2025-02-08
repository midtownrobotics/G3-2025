package frc.robot.subsystems.elevator.winch;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class WinchIOSim implements WinchIO {
  @Override
  public void updateInputs(WinchInputs inputs) {
  }

  @Override
  public void setScorePosition(Distance position) {
  }

  @Override
  public void setClimbPosition(Distance position) {
  }

  @Override
  public Distance getPosition() {
    return Inches.zero();
  }
}

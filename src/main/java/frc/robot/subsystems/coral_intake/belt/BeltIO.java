package frc.robot.subsystems.coral_intake.belt;

import frc.lib.team1648.NeoInputs;
import org.littletonrobotics.junction.AutoLog;

public interface BeltIO {

  @AutoLog
  public class BeltInputs extends NeoInputs {}

  /**
   * Set motor output voltage
   *
   * @param voltage
   */
  public void setVoltage(int voltage);
  /**
   * Update input class for Belt
   *
   * @param inputs
   */
  public void updateInputs(BeltInputs inputs);
}

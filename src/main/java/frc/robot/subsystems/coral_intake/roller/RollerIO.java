package frc.robot.subsystems.coral_intake.roller;

import frc.lib.team1648.NeoInputs;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerInputs extends NeoInputs {}

  /**
   * Set motor output voltage
   *
   * @param voltage
   */
  public void setVoltage(int voltage);
  /**
   * Update input class for Roller
   *
   * @param inputs
   */
  public void updateInputs(RollerInputs inputs);
}

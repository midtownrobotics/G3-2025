package frc.robot.subsystems.algae_claw.roller;

import frc.lib.team1648.KrakenInputs;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerInputs extends KrakenInputs {}

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

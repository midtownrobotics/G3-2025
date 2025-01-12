package frc.robot.subsystems.coral_outtake.roller;

import frc.lib.team1648.KrakenInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerInputs {
    public KrakenInputsAutoLogged motorInputs;
  }
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

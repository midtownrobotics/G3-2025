package frc.robot.subsystems.elevator.winch;

import edu.wpi.first.units.measure.Distance;
import frc.lib.team1648.KrakenInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public class WinchInputs {
    public KrakenInputsAutoLogged left;
    public KrakenInputsAutoLogged right;
  }

  /**
   * Updates PID loop to follow new target position
   *
   * @param position
   */
  public void setPosition(Distance position);

  /**
   * Updates input class for Winch
   *
   * @param inputs
   */
  public void updateInputs(WinchInputs inputs);
}

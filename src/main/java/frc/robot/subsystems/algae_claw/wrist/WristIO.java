package frc.robot.subsystems.algae_claw.wrist;

import edu.wpi.first.units.measure.Angle;
import frc.lib.team1648.KrakenInputs;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public class WristInputs extends KrakenInputs {
    public Angle wristPosition;
  }

  /** sets position */
  public void setPosition(Angle position);

  /**
   * updates input class for Wrist
   *
   * @param inputs
   */
  public void updateInputs(WristInputs inputs);
}

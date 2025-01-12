package frc.robot.subsystems.coral_intake.pivot;

import edu.wpi.first.units.measure.Angle;
import frc.lib.team1648.NeoInputs;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public class PivotInputs extends NeoInputs {
    public Angle pivotPosition;
  }

  /** sets position */
  public void setPosition(Angle position);
  /**
   * Update input class for Pivot
   *
   * @param inputs
   */
  public void updateInputs(PivotInputs inputs);
}

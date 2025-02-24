package frc.robot.subsystems.coral_intake.pivot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public class PivotInputs {
    public Angle position = Units.Radians.zero();
    public AngularVelocity velocity = Units.RPM.zero();
    public Voltage appliedVoltage = Units.Volts.zero();
    public Current supplyCurrent = Units.Amps.zero();
    public Temperature temperature = Units.Fahrenheit.zero();

    public Angle absolutePosition = Units.Radians.zero();
  }

  /** Sets the motor voltage. */
  public void setVoltage(Voltage voltage);

  /**
   * Update input class for Pivot
   *
   * @param inputs
   */
  public void updateInputs(PivotInputs inputs);
}

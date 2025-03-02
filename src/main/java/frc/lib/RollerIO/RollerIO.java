package frc.lib.RollerIO;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerInputs {
    public boolean connected;
    public Angle position = Units.Radians.zero();
    public AngularVelocity velocity = Units.RPM.zero();
    public Voltage appliedVoltage = Units.Volts.zero();
    public Current supplyCurrent = Units.Amps.zero();
    public Current torqueCurrent = Units.Amps.zero();
    public Temperature temperature = Units.Fahrenheit.zero();
  }

  /**
   * Set motor output voltage
   *
   * @param voltage
   */
  public void setVoltage(Voltage voltage);

  public void setOutput(double output);

  /**
   * Update input class for Roller
   *
   * @param inputs
   */
  public void updateInputs(RollerInputs inputs);
}

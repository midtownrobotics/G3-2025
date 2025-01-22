package frc.robot.subsystems.algae_claw.roller;

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
    public boolean connected = true;
    public Angle position = Units.Radians.zero();
    public AngularVelocity velocity = Units.RPM.zero();
    public Voltage appliedVoltage = Units.Volt.zero();
    public Current supplyCurrent = Units.Amp.zero();
    public Current torqueCurrent = Units.Amp.zero();
    public Temperature temperature = Units.Kelvin.zero();
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

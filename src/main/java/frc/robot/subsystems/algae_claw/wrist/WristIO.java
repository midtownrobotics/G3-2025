package frc.robot.subsystems.algae_claw.wrist;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public class WristInputs {
    public boolean connected = true;
    public Angle position = Units.Radians.zero();
    public AngularVelocity velocity = Units.RPM.zero();
    public Voltage appliedVoltage = Units.Volt.zero();
    public Current supplyCurrent = Units.Amp.zero();
    public Current torqueCurrent = Units.Amp.zero();
    public Temperature temperature = Units.Kelvin.zero();

    public Angle absolutePosition = Units.Radians.zero();
    public AngularVelocity absoluteVelocity = Units.RPM.zero();
  }

  /** sets position */
  public void setPosition(Angle position);

  /**
   * updates input class for Wrist
   *
   * @param inputs
   */
  public void updateInputs(WristInputs inputs);

  /**
   * returns encoder position
   *
   * @return
   */
  public double getEncoderPosition();

}

package frc.lib.team1648;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class KrakenInputs {
  public AngularVelocity velocity = Units.RPM.zero();
  public Temperature tempFahrenheit = Units.Fahrenheit.zero();
  public Voltage outputVoltage = Units.Volt.zero();
  public Current current = Units.Amp.zero();

  /**
   * Updates inputs for kraken motor
   *
   * @param kraken
   */
  public void updateInputs(TalonFX kraken) {
    velocity = kraken.getVelocity().getValue();
    tempFahrenheit = kraken.getDeviceTemp().getValue();
    outputVoltage = kraken.getMotorVoltage().getValue();
    current = kraken.getSupplyCurrent().getValue();
  }
}

package frc.robot.subsystems.elevator.winch;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public class WinchMotorInputs {
    public boolean connected = true;
    public Angle position = Units.Radians.zero();
    public AngularVelocity velocity = Units.RPM.zero();
    public Voltage appliedVoltage = Units.Volt.zero();
    public Current supplyCurrent = Units.Amp.zero();
    public Current torqueCurrent = Units.Amp.zero();
    public Temperature tempFahrenheit = Units.Fahrenheit.zero();
  }

  @AutoLog
  public class WinchInputs {
    public WinchMotorInputsAutoLogged left = new WinchMotorInputsAutoLogged();
    public WinchMotorInputsAutoLogged right = new WinchMotorInputsAutoLogged();
    public Angle absolutePosition = Units.Radians.zero();
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

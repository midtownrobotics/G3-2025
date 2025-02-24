package frc.robot.subsystems.elevator.winch;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public class WinchMotorInputs {
    public boolean connected = true;
    public Distance position = Units.Inches.zero();
    public LinearVelocity velocity = Units.InchesPerSecond.zero();
    public Voltage appliedVoltage = Units.Volt.zero();
    public Current supplyCurrent = Units.Amp.zero();
    public Current torqueCurrent = Units.Amp.zero();
    public Temperature temperature = Units.Kelvin.zero();
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
  public void setScorePosition(Distance position);

  /**
   * Updates PID loop to follow new target position
   *
   * @param position
   */
  public void setClimbPosition(Distance position);

  /**
   * Sets the motor voltage.
   */
  public void setVoltage(Voltage voltage);

  /**
   * Updates input class for Winch
   *
   * @param inputs
   */
  public void updateInputs(WinchInputs inputs);
}

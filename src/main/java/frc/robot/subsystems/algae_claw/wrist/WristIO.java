package frc.robot.subsystems.algae_claw.wrist;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.dashboard.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

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

  public class WristConfig {
    private LoggedTunableNumber p, i, d, s, g, a, v,
      currentLimit, currentLowerLimit, currentLowerTime;
    private LoggedTunableNumber[] constants = new LoggedTunableNumber[]{p, i, d, s, g, a, v,
      currentLimit, currentLowerLimit, currentLowerTime};

    /**
     * constructor for wrist pid config object
     */
    public WristConfig() {
      p = new LoggedTunableNumber("WristIOKraken/P", 0);
      i = new LoggedTunableNumber("WristIOKraken/I", 0);
      d = new LoggedTunableNumber("WristIOKraken/D", 0);
      s = new LoggedTunableNumber("WristIOKraken/S", 0);
      g = new LoggedTunableNumber("WristIOKraken/G", 0);
      a = new LoggedTunableNumber("WristIOKraken/A", 0);
      v = new LoggedTunableNumber("WristIOKraken/V", 0);
      currentLimit = new LoggedTunableNumber("WristIOKraken/currentLimit", 0);
      currentLowerLimit = new LoggedTunableNumber("WristIOKraken/currentLowerLimit", 0);
      currentLowerTime = new LoggedTunableNumber("WristIOKraken/currentLowerTime", 0);
    }

    /**
     * applies constants to slot 0 of specific motor
     * @param motor
     */
    public void apply(TalonFX motor) {
      TalonFXConfiguration configuration = new TalonFXConfiguration();
      configuration.Slot0 = new Slot0Configs()
        .withKP(p.get())
        .withKI(i.get())
        .withKD(d.get())
        .withKS(s.get())
        .withKG(g.get())
        .withKA(a.get())
        .withKV(v.get());
      
        configuration.CurrentLimits = new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(currentLimit.get())
          .withSupplyCurrentLowerLimit(currentLowerLimit.get())
          .withSupplyCurrentLowerTime(currentLowerTime.get());

      motor.getConfigurator().apply(configuration);
    }

    /**
     * returns logged tunable numbers in config object
     * @return logged tunable numbers in config object
     */
    public LoggedTunableNumber[] getConstants() {
        return constants;
    }
  }

  

  /** sets position */
  public void setPosition(Angle position);

  /** sets voltage */
  public void setVoltage(Voltage volts);

  /**
   * updates input class for Wrist
   *
   * @param inputs
   */
  public void updateInputs(WristInputs inputs);
}

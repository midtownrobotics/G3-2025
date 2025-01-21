package frc.robot.subsystems.algae_claw.roller;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.Constants;
import lombok.Getter;

public class RollerIOKraken implements RollerIO {

  @Getter private TalonFX roller;
  @Getter private StatusSignal<Angle> position;
  @Getter private StatusSignal<AngularVelocity> velocity;
  @Getter private StatusSignal<Voltage> voltage;
  @Getter private StatusSignal<Current> supplyCurrent;
  @Getter private StatusSignal<Current> torqueCurrent;
  @Getter private StatusSignal<Temperature> temperature;

  /** Constructor for rollerIO for kraken motors. */
  public RollerIOKraken(int rollerID) {
    roller = new TalonFX(rollerID);
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();

    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(Constants.KRAKEN_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(1);

    roller.getConfigurator().apply(krakenConfig);

    position = roller.getPosition();
    velocity = roller.getVelocity();
    voltage = roller.getMotorVoltage();
    supplyCurrent = roller.getSupplyCurrent();
    torqueCurrent = roller.getTorqueCurrent();
    temperature = roller.getDeviceTemp();

    position.setUpdateFrequency(50);
    velocity.setUpdateFrequency(50);
    voltage.setUpdateFrequency(50);
    supplyCurrent.setUpdateFrequency(50);
    torqueCurrent.setUpdateFrequency(50);
    temperature.setUpdateFrequency(50);
    
    roller.optimizeBusUtilization();
  }


  @Override
  public void setVoltage(int voltage) {
    roller.setVoltage(voltage);
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    inputs.connected = roller.isConnected();
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = voltage.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();
  }
}

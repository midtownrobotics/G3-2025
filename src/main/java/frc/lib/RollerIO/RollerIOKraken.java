package frc.lib.RollerIO;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;

public class RollerIOKraken implements RollerIO {

  private TalonFX motor;
  private StatusSignal<Angle> position;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> voltage;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Current> torqueCurrent;
  private StatusSignal<Temperature> temperature;

  /** Constructor for rollerIO for kraken motors. */
  public RollerIOKraken(int motorID, CANBusStatusSignalRegistration bus) {
    motor = new TalonFX(motorID);
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();

    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(Constants.KRAKEN_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(1);

    motor.getConfigurator().apply(krakenConfig);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    temperature = motor.getDeviceTemp();

    position.setUpdateFrequency(50);
    velocity.setUpdateFrequency(50);
    voltage.setUpdateFrequency(50);
    supplyCurrent.setUpdateFrequency(50);
    torqueCurrent.setUpdateFrequency(50);
    temperature.setUpdateFrequency(50);

    bus
      .register(position)
      .register(velocity)
      .register(voltage)
      .register(supplyCurrent)
      .register(torqueCurrent)
      .register(temperature);

    motor.optimizeBusUtilization();
  }


  @Override
  public void setVoltage(Voltage voltage) {
    VoltageOut request = new VoltageOut(voltage);
    motor.setControl(request);
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    inputs.connected = motor.isConnected();
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = voltage.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();
  }
}

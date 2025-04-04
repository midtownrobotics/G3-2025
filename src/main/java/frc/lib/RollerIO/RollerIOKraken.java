package frc.lib.RollerIO;

import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    this(motorID, bus, false);
  }

  /** Constructor for rollerIO for kraken motors with an invert. */
  public RollerIOKraken(int motorID, CANBusStatusSignalRegistration bus, boolean invert) {
    motor = new TalonFX(motorID, bus.getCanBusId());
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();

    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
          .withSupplyCurrentLimitEnable(true);

    krakenConfig.MotorOutput.withInverted(invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);

    krakenConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    motor.getConfigurator().apply(krakenConfig);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    temperature = motor.getDeviceTemp();

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
      50,
      position,
      velocity,
      voltage,
      supplyCurrent,
      torqueCurrent,
      temperature
    ));

    bus
      .register(position)
      .register(velocity)
      .register(voltage)
      .register(supplyCurrent)
      .register(torqueCurrent)
      .register(temperature);

    tryUntilOk(5, () -> motor.optimizeBusUtilization());
  }


  @Override
  public void setVoltage(Voltage voltage) {
    ControlRequest request = new VoltageOut(voltage);
    motor.setControl(request);
  }

  @Override
  public void setOutput(double output) {
      motor.set(output);
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

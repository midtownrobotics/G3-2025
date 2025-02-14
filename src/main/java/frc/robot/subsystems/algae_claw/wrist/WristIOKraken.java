package frc.robot.subsystems.algae_claw.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.algae_claw.AlgaeClawConstants;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import lombok.Getter;

public class WristIOKraken implements WristIO {

  @Getter private TalonFX wristMotor;
  private DutyCycleEncoder encoder;
  private final double GEAR_RATIO = 90;

  @Getter private StatusSignal<Angle> position;
  @Getter private StatusSignal<AngularVelocity> velocity;
  @Getter private StatusSignal<Voltage> voltage;
  @Getter private StatusSignal<Current> supplyCurrent;
  @Getter private StatusSignal<Current> torqueCurrent;
  @Getter private StatusSignal<Temperature> temperature;
  @Getter private StatusSignal<Double> dutyCycle;

  /** Constructor for wristIO for kraken motors. */
  public WristIOKraken(int wristMotorID, int encoderID, CANBusStatusSignalRegistration bus) {
    wristMotor = new TalonFX(wristMotorID);
    encoder = new DutyCycleEncoder(new DigitalInput(encoderID));

    configureMotor();

    wristMotor.setPosition(encoder.get());

    velocity = wristMotor.getVelocity();
    voltage = wristMotor.getMotorVoltage();
    supplyCurrent = wristMotor.getSupplyCurrent();
    torqueCurrent = wristMotor.getTorqueCurrent();
    temperature = wristMotor.getDeviceTemp();
    dutyCycle = wristMotor.getDutyCycle();

    bus
       .register(position)
       .register(velocity)
       .register(voltage)
       .register(torqueCurrent)
       .register(supplyCurrent)
       .register(temperature)
       .register(dutyCycle);

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      position,
      velocity,
      voltage,
      torqueCurrent,
      supplyCurrent,
      temperature,
      dutyCycle
    ));

    tryUntilOk(5, () -> wristMotor.optimizeBusUtilization(0, 1));

    configureMotor();
  }

  @Override
  public void setPosition(Angle position) {
    PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(position);
    wristMotor.setControl(request);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    inputs.connected = wristMotor.isConnected();
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = voltage.getValue().times(wristMotor.getDutyCycle(true).getValue());
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();
    inputs.absolutePosition = Rotations.of(encoder.get());

    updateConstants();
  }

  private void updateConstants() {
    LoggedTunableNumber.ifChanged(
      hashCode(),
      this::configureMotor,
      AlgaeClawConstants.PID.p,
      AlgaeClawConstants.PID.i,
      AlgaeClawConstants.PID.d,
      AlgaeClawConstants.PID.s,
      AlgaeClawConstants.PID.v,
      AlgaeClawConstants.PID.g,
      AlgaeClawConstants.PID.a
    );
  }

  private void configureMotor() {
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();

    krakenConfig.Slot0 =
          new Slot0Configs()
              .withKP(AlgaeClawConstants.PID.p.get())
              .withKI(AlgaeClawConstants.PID.i.get())
              .withKD(AlgaeClawConstants.PID.d.get())
              .withKS(AlgaeClawConstants.PID.s.get())
              .withKG(AlgaeClawConstants.PID.g.get())
              .withKV(AlgaeClawConstants.PID.v.get())
              .withKA(AlgaeClawConstants.PID.a.get())
              .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT);
    wristMotor.getConfigurator().apply(krakenConfig);
  }

  @Override
  public void setVoltage(Voltage volts) {
    wristMotor.setVoltage(volts.in(Volts));
  }
}

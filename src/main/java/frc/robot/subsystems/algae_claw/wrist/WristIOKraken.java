package frc.robot.subsystems.algae_claw.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.CANBusStatusSignalRegistration;
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
  private WristConfig wristConfig;

  /** Constructor for wristIO for kraken motors. */
  public WristIOKraken(int wristMotorID, int encoderID, CANBusStatusSignalRegistration bus) {
    wristConfig = new WristConfig();
    wristMotor = new TalonFX(wristMotorID);
    encoder = new DutyCycleEncoder(new DigitalInput(encoderID));

    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    krakenConfig.Feedback = new FeedbackConfigs()
      .withSensorToMechanismRatio(GEAR_RATIO);
    // Shooting Slot
    wristConfig.apply(wristMotor);

    wristMotor.getConfigurator().apply(krakenConfig);

    wristMotor.setPosition(encoder.get());

    velocity = wristMotor.getVelocity();
    voltage = wristMotor.getMotorVoltage();
    supplyCurrent = wristMotor.getSupplyCurrent();
    torqueCurrent = wristMotor.getTorqueCurrent();
    temperature = wristMotor.getDeviceTemp();
    dutyCycle = wristMotor.getDutyCycle();

    position.setUpdateFrequency(50);
    velocity.setUpdateFrequency(50);
    voltage.setUpdateFrequency(50);
    supplyCurrent.setUpdateFrequency(50);
    torqueCurrent.setUpdateFrequency(50);
    temperature.setUpdateFrequency(50);
    dutyCycle.setUpdateFrequency(50);

    tryUntilOk(5, () -> wristMotor.optimizeBusUtilization(1));

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
  }

  @Override
  public void applyConfig() {
      wristConfig.apply(wristMotor);
  }

  @Override
  public WristConfig getConfig() {
      return wristConfig;
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
  }
}

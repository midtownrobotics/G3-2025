package frc.robot.subsystems.coral_outtake_pivot.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.coral_outtake_pivot.CoralOuttakePivotConstants;
import frc.robot.utils.CANBusStatusSignalRegistration;
import lombok.Getter;

public class OuttakePivotIOKraken implements OuttakePivotIO {

  @Getter private TalonFX motor;

  @Getter private DutyCycleEncoder encoder;

  @Getter private StatusSignal<Voltage> voltage;
  @Getter private StatusSignal<Current> supplyCurrent;
  @Getter private StatusSignal<Current> torqueCurrent;
  @Getter private StatusSignal<Temperature> temperature;
  @Getter private StatusSignal<Angle> position;
  @Getter private StatusSignal<AngularVelocity> velocity;

  private final CANBusStatusSignalRegistration bus;

  /** Contructor for real winch with Krakens */
  public OuttakePivotIOKraken(int motorID,
                       int encoderID,
                       CANBusStatusSignalRegistration bus) {

    this.bus = bus;

    motor = new TalonFX(motorID, bus.getCanBusId());

    encoder = new DutyCycleEncoder(encoderID);

    Angle initialPosition = getInitialAngle();

    tryUntilOk(5, () -> motor.setPosition(initialPosition));

    voltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    temperature = motor.getDeviceTemp();
    position = motor.getPosition();
    velocity = motor.getVelocity();

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50,
                                                                              voltage,
                                                                              supplyCurrent,
                                                                              torqueCurrent,
                                                                              temperature,
                                                                              position,
                                                                              velocity));

    tryUntilOk(5, () -> motor.optimizeBusUtilization(0, 1));

    bus
      .register(voltage)
      .register(supplyCurrent)
      .register(torqueCurrent)
      .register(temperature);
  }

  @Override
  public void updateInputs(OuttakePivotInputs inputs) {
    bus.refreshSignals();

    inputs.absolutePosition = getAbsoluteEncoderPosition();
    inputs.zeroedPosition = getZeroedAbsoluteEncoderPosition();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = voltage.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();

    Logger.recordOutput("CoralOuttake/ZeroedAbsoluteEncoder", getZeroedAbsoluteEncoderPosition().in(Degrees));
    Logger.recordOutput("CoralOuttake/InitialAngle", getInitialAngle().in(Degrees));
    Logger.recordOutput("CoralOuttake/AbsoluteEncoderValueDeg", getAbsoluteEncoderPosition().in(Degrees));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.in(Volts));
  }

  private Angle getInitialAngle() {
    return getZeroedAbsoluteEncoderPosition().times(CoralOuttakePivotConstants.coralOuttakePivotGearRatio);
  }

  private Angle getZeroedAbsoluteEncoderPosition() {
    return getAbsoluteEncoderPosition().minus(CoralOuttakePivotConstants.absoluteEncoderOffset);
  }

  private Angle getAbsoluteEncoderPosition() {
    return Rotations.of(encoder.get());
  }
}

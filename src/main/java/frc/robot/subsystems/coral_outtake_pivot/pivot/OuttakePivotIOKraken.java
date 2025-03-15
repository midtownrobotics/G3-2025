package frc.robot.subsystems.coral_outtake_pivot.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.subsystems.coral_outtake_pivot.CoralOuttakePivotConstants;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class OuttakePivotIOKraken implements OuttakePivotIO {

  @Getter private TalonFX motor;

  @Getter private DutyCycleEncoder encoder;

  @Getter private StatusSignal<Voltage> voltage;
  @Getter private StatusSignal<Current> supplyCurrent;
  @Getter private StatusSignal<Current> torqueCurrent;
  @Getter private StatusSignal<Temperature> temperature;

  /** Contructor for real winch with Krakens */
  public OuttakePivotIOKraken(int motorID,
                       int encoderID,
                       CANBusStatusSignalRegistration bus) {

    motor = new TalonFX(motorID, "Elevator");

    encoder = new DutyCycleEncoder(encoderID);

    Angle position = getInitialAngle();

    tryUntilOk(5, () -> motor.setPosition(position));

    configureMotors();

    voltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();
    temperature = motor.getDeviceTemp();

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50,
                                                                              voltage,
                                                                              supplyCurrent,
                                                                              torqueCurrent,
                                                                              temperature));

    tryUntilOk(5, () -> motor.optimizeBusUtilization(0, 1));

    bus
      .register(voltage)
      .register(supplyCurrent)
      .register(torqueCurrent)
      .register(temperature);
  }

  /**
   * Sets the setpoint of the kraken winch.
   * @param position Setpoint to set.
   */
  public void setPosition(Angle position) {
    MotionMagicVoltage request = new MotionMagicVoltage(position.times(CoralOuttakePivotConstants.coralOuttakePivotGearRatio));
    motor.setControl(request);
  }

  @Override
  public void updateInputs(OuttakePivotInputs inputs) {
    inputs.absolutePosition = getAbsoluteEncoderPosition();
    inputs.position = motor.getPosition().getValue();
    inputs.velocity = motor.getVelocity().getValue();
    inputs.appliedVoltage = voltage.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();

    Logger.recordOutput("CoralOuttake/ZeroedAbsoluteEncoder", getZeroedAbsoluteEncoderPosition().in(Degrees));
    Logger.recordOutput("CoralOuttake/InitialAngle", getInitialAngle().in(Degrees));

    updateConstants();
  }

  private void updateConstants() {
    LoggedTunableNumber.ifChanged(
      hashCode(),
      this::configureMotors,
      // Score
      CoralOuttakePivotConstants.PID.p,
      CoralOuttakePivotConstants.PID.i,
      CoralOuttakePivotConstants.PID.d,
      CoralOuttakePivotConstants.PID.s,
      CoralOuttakePivotConstants.PID.v,
      CoralOuttakePivotConstants.PID.g
    );
  }

  private void configureMotors() {
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    // Scoring Slot
    krakenConfig.Slot0 =
        new Slot0Configs()
            .withKP(CoralOuttakePivotConstants.PID.p.get())
            .withKI(CoralOuttakePivotConstants.PID.i.get())
            .withKD(CoralOuttakePivotConstants.PID.d.get())
            .withKS(CoralOuttakePivotConstants.PID.s.get())
            .withKG(CoralOuttakePivotConstants.PID.g.get())
            .withKV(CoralOuttakePivotConstants.PID.v.get())
            .withGravityType(GravityTypeValue.Elevator_Static);
    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    krakenConfig.MotionMagic.withMotionMagicCruiseVelocity(CoralOuttakePivotConstants.PID.maxPivotV);
    krakenConfig.MotionMagic.withMotionMagicAcceleration(CoralOuttakePivotConstants.PID.maxPivotA);

    tryUntilOk(5, () -> motor.getConfigurator().apply(krakenConfig));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.in(Units.Volts));
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

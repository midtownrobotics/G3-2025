package frc.robot.subsystems.coral_outtake.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import frc.robot.subsystems.coral_outtake.CoralOuttakeConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
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

    // Angle position = getInitialAngle();

    tryUntilOk(5, () -> motor.setPosition(Degrees.zero()));

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
    MotionMagicVoltage request = new MotionMagicVoltage(position);
    motor.setControl(request);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
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
      CoralOuttakeConstants.PID.p,
      CoralOuttakeConstants.PID.i,
      CoralOuttakeConstants.PID.d,
      CoralOuttakeConstants.PID.s,
      CoralOuttakeConstants.PID.v,
      CoralOuttakeConstants.PID.g
    );
  }

  private void configureMotors() {
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    // Scoring Slot
    krakenConfig.Slot0 =
        new Slot0Configs()
            .withKP(ElevatorConstants.PID_SCORE.p.get())
            .withKI(ElevatorConstants.PID_SCORE.i.get())
            .withKD(ElevatorConstants.PID_SCORE.d.get())
            .withKS(ElevatorConstants.PID_SCORE.s.get())
            .withKG(ElevatorConstants.PID_SCORE.g.get())
            .withKA(ElevatorConstants.PID_SCORE.a.get())
            .withKV(ElevatorConstants.PID_SCORE.v.get())
            .withGravityType(GravityTypeValue.Elevator_Static);
    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    krakenConfig.MotionMagic.withMotionMagicCruiseVelocity(CoralOuttakeConstants.PID.maxPivotV);
    krakenConfig.MotionMagic.withMotionMagicAcceleration(CoralOuttakeConstants.PID.maxPivotA);

    tryUntilOk(5, () -> motor.getConfigurator().apply(krakenConfig));
    tryUntilOk(5, () -> motor.setControl(new Follower(motor.getDeviceID(), false)));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage.in(Units.Volts));
  }

  private Angle getInitialAngle() {
    double radians = (getZeroedAbsoluteEncoderPosition().in(Radians) + 2 * Math.PI + 0.2) % (2 * Math.PI) - 0.2;

    return Radians.of(radians).times(ElevatorConstants.kGearing);
  }

  private Angle getZeroedAbsoluteEncoderPosition() {
    return getAbsoluteEncoderPosition().minus(ElevatorConstants.absoluteEncoderOffset);
  }

  private Angle getAbsoluteEncoderPosition() {
    return Rotations.of(encoder.get());
  }
}

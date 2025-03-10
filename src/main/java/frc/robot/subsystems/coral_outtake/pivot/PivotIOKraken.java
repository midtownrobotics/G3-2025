package frc.robot.subsystems.coral_outtake.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class PivotIOKraken implements PivotIO {

  @Getter private TalonFX motor;

  @Getter private DutyCycleEncoder encoder;

  @Getter private StatusSignal<Voltage> voltage;
  @Getter private StatusSignal<Current> supplyCurrent;
  @Getter private StatusSignal<Current> torqueCurrent;
  @Getter private StatusSignal<Temperature> temperature;

  /** Contructor for real winch with Krakens */
  public PivotIOKraken(int motorID,
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

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      voltage,
      supplyCurrent,
      torqueCurrent,
      temperature
      ));
  }

  /**
   * Sets the setpoint of the kraken winch.
   * @param position Setpoint to set.
   * @param slot PID slot to set to.
   */
  public void setPosition(Distance position, int slot) {
    MotionMagicVoltage request = new MotionMagicVoltage(meterToRotation(position)).withSlot(slot);
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
      ElevatorConstants.PID_SCORE.p,
      ElevatorConstants.PID_SCORE.i,
      ElevatorConstants.PID_SCORE.d,
      ElevatorConstants.PID_SCORE.s,
      ElevatorConstants.PID_SCORE.v,
      ElevatorConstants.PID_SCORE.g,
      ElevatorConstants.PID_SCORE.a,
      // Climb
      ElevatorConstants.PID_CLIMB.p,
      ElevatorConstants.PID_CLIMB.i,
      ElevatorConstants.PID_CLIMB.d,
      ElevatorConstants.PID_CLIMB.s,
      ElevatorConstants.PID_CLIMB.v,
      ElevatorConstants.PID_CLIMB.g,
      ElevatorConstants.PID_CLIMB.a
    );
  }

  /**
   * Converts distance unit to kraken rotations
   *
   * @param m
   * @return
   */
  public Angle meterToRotation(Distance m) {
    return m.div(Inches.of(6.75)).times(Rotations.of(ElevatorConstants.kGearing)).div(3);
  }

  /**
   * Converts rotational velocity (rotations per second) to linear velocity (meters per second)
   *
   * @param velocity Rotational velocity
   * @return Linear velocity
   */
  public LinearVelocity rotationToLinearVelocity(AngularVelocity velocity) {
    return Units.MetersPerSecond.of((6.75 * velocity.in(Units.RotationsPerSecond)) / ElevatorConstants.kGearing).times(3);
  }

  /**
   * Converts kraken rotation to distance
   *
   * @param a
   * @return
   */
  public Distance rotationToDistance(Angle a) {
    return a.div(Rotations.of(ElevatorConstants.kGearing)).times(Inches.of(6.75)).times(3);
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
    // Climbing Slot
    krakenConfig.Slot1 =
        new Slot1Configs()
            .withKP(ElevatorConstants.PID_CLIMB.p.get())
            .withKI(ElevatorConstants.PID_CLIMB.i.get())
            .withKD(ElevatorConstants.PID_CLIMB.d.get())
            .withKS(ElevatorConstants.PID_CLIMB.s.get())
            .withKG(ElevatorConstants.PID_CLIMB.g.get())
            .withKA(ElevatorConstants.PID_CLIMB.a.get())
            .withKV(ElevatorConstants.PID_CLIMB.v.get())
            .withGravityType(GravityTypeValue.Elevator_Static);
    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    krakenConfig.MotionMagic.withMotionMagicCruiseVelocity(meterToRotation(Feet.of(10)).per(Second));
    krakenConfig.MotionMagic.withMotionMagicAcceleration(meterToRotation(Feet.of(10)).per(Second).per(Second));

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

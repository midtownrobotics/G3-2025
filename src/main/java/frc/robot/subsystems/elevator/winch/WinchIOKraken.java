package frc.robot.subsystems.elevator.winch;

import static frc.robot.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import lombok.Getter;

public class WinchIOKraken implements WinchIO {

  private static final double GEARING = 12;
  private static final Distance WHEEL_RADIUS = Units.Inches.of(1);

  @Getter private TalonFX leftMotor;
  @Getter private TalonFX rightMotor;

  @Getter private StatusSignal<Angle> leftPosition;
  @Getter private StatusSignal<AngularVelocity> leftVelocity;
  @Getter private StatusSignal<Voltage> leftVoltage;
  @Getter private StatusSignal<Current> leftSupplyCurrent;
  @Getter private StatusSignal<Current> leftTorqueCurrent;
  @Getter private StatusSignal<Temperature> leftTemperature;

  @Getter private StatusSignal<Angle> rightPosition;
  @Getter private StatusSignal<AngularVelocity> rightVelocity;
  @Getter private StatusSignal<Voltage> rightVoltage;
  @Getter private StatusSignal<Current> rightSupplyCurrent;
  @Getter private StatusSignal<Current> rightTorqueCurrent;
  @Getter private StatusSignal<Temperature> rightTemperature;

  /** Contructor for real winch with Krakens */
  public WinchIOKraken(int leftMotorID, int rightMotorID, CANBusStatusSignalRegistration bus) {

    leftMotor = new TalonFX(leftMotorID);
    rightMotor = new TalonFX(rightMotorID);
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    // Scoring Slot
    krakenConfig.Slot0 =
        new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKA(0)
            .withKV(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // Climbing Slot
    krakenConfig.Slot1 =
        new Slot1Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKA(0)
            .withKV(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerTime(1);
    krakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    new Rotation2d();
    leftMotor.getConfigurator().apply(krakenConfig);
    rightMotor.getConfigurator().apply(krakenConfig);
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));

    leftPosition = leftMotor.getPosition();
    leftVelocity = leftMotor.getVelocity();
    leftVoltage = leftMotor.getMotorVoltage();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    leftTorqueCurrent = leftMotor.getTorqueCurrent();
    leftTemperature = leftMotor.getDeviceTemp();

    leftPosition.setUpdateFrequency(50);
    leftVelocity.setUpdateFrequency(50);
    leftVoltage.setUpdateFrequency(50);
    leftSupplyCurrent.setUpdateFrequency(50);
    leftTorqueCurrent.setUpdateFrequency(50);
    leftTemperature.setUpdateFrequency(50);

    leftMotor.optimizeBusUtilization();

    rightPosition = rightMotor.getPosition();
    rightVelocity = rightMotor.getVelocity();
    rightVoltage = rightMotor.getMotorVoltage();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();
    rightTorqueCurrent = rightMotor.getTorqueCurrent();
    rightTemperature = rightMotor.getDeviceTemp();

    rightPosition.setUpdateFrequency(50);
    rightVelocity.setUpdateFrequency(50);
    rightVoltage.setUpdateFrequency(50);
    rightSupplyCurrent.setUpdateFrequency(50);
    rightTorqueCurrent.setUpdateFrequency(50);
    rightTemperature.setUpdateFrequency(50);

    rightMotor.optimizeBusUtilization();

    bus
      .register(rightPosition)
      .register(rightVelocity)
      .register(rightVoltage)
      .register(rightSupplyCurrent)
      .register(rightTorqueCurrent)
      .register(rightTemperature)
      .register(leftPosition)
      .register(leftVelocity)
      .register(leftVoltage)
      .register(leftSupplyCurrent)
      .register(leftTorqueCurrent)
      .register(leftTemperature);

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      rightPosition,
      rightVelocity,
      rightVoltage,
      rightSupplyCurrent,
      rightTorqueCurrent,
      rightTemperature,
      leftPosition,
      leftVelocity,
      leftVoltage,
      leftSupplyCurrent,
      leftTorqueCurrent,
      leftTemperature
      ));
  }

  @Override
  public void setScorePosition(Distance position) {
    setPosition(position, 0);
  }

  @Override
  public void setClimbPosition(Distance position) {
    setPosition(position, 1);
  }

  /**
   * Sets the setpoint of the kraken winch.
   * @param position Setpoint to set.
   * @param slot PID slot to set to.
   */
  public void setPosition(Distance position, int slot) {
    double p = meterToRotation(position);

    PositionTorqueCurrentFOC leftRequest = new PositionTorqueCurrentFOC(p).withSlot(slot);
    leftMotor.setControl(leftRequest);
  }

  @Override
  public void updateInputs(WinchInputs inputs) {
    inputs.left.connected = leftMotor.isConnected();
    inputs.left.position = leftPosition.getValue();
    inputs.left.velocity = leftVelocity.getValue();
    inputs.left.appliedVoltage = leftVoltage.getValue();
    inputs.left.supplyCurrent = leftSupplyCurrent.getValue();
    inputs.left.torqueCurrent = leftTorqueCurrent.getValue();
    inputs.left.temperature = leftTemperature.getValue();

    inputs.right.connected = leftMotor.isConnected();
    inputs.right.position = leftPosition.getValue();
    inputs.right.velocity = leftVelocity.getValue();
    inputs.right.appliedVoltage = leftVoltage.getValue();
    inputs.right.supplyCurrent = leftSupplyCurrent.getValue();
    inputs.right.torqueCurrent = leftTorqueCurrent.getValue();
    inputs.right.temperature = leftTemperature.getValue();
  }

  @Override
  public Distance getPosition() {
    return rotationToDistance(leftMotor.getPosition().getValue());
  }

  /**
   * Converts distance unit to kraken rotations
   *
   * @param m
   * @return
   */
  public double meterToRotation(Distance m) {
    return m.in(Units.Meter) / (2 * Math.PI * WHEEL_RADIUS.in(Units.Meter)) * GEARING;
  }

  /**
   * Converts kraken rotation to distance
   *
   * @param a
   * @return
   */
  public Distance rotationToDistance(Angle a) {
    return Units.Meters.of((2 * Math.PI * WHEEL_RADIUS.in(Units.Meter) * a.in(Units.Rotation)) / GEARING);
  }
}

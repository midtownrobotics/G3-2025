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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class WinchIOKraken implements WinchIO {

  private static final double GEARING = 12;
  private static final Distance WHEEL_RADIUS = Units.Inches.of(1);

  @Getter private TalonFX leftMotor;
  @Getter private TalonFX rightMotor;

  @Getter private DutyCycleEncoder leftMotorEncoder;
  @Getter private DutyCycleEncoder rightMotorEncoder;

  @Getter private StatusSignal<Voltage> leftVoltage;
  @Getter private StatusSignal<Current> leftSupplyCurrent;
  @Getter private StatusSignal<Current> leftTorqueCurrent;
  @Getter private StatusSignal<Temperature> leftTemperature;

  @Getter private StatusSignal<Voltage> rightVoltage;
  @Getter private StatusSignal<Current> rightSupplyCurrent;
  @Getter private StatusSignal<Current> rightTorqueCurrent;
  @Getter private StatusSignal<Temperature> rightTemperature;

  /** Contructor for real winch with Krakens */
  public WinchIOKraken(int leftMotorID, 
                       int rightMotorID, 
                       int leftMotorEncoderID, 
                       int rightMotorEncoderID,
                       CANBusStatusSignalRegistration bus) {

    leftMotor = new TalonFX(leftMotorID);
    rightMotor = new TalonFX(rightMotorID);

    leftMotorEncoder = new DutyCycleEncoder(rightMotorID);
    rightMotorEncoder = new DutyCycleEncoder(rightMotorID);

    tryUntilOk(5, () -> leftMotor.setPosition(leftMotorEncoder.get()));
    tryUntilOk(5, () -> rightMotor.setPosition(rightMotorEncoder.get()));
    
    configureMotors();

    leftVoltage = leftMotor.getMotorVoltage();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    leftTorqueCurrent = leftMotor.getTorqueCurrent();
    leftTemperature = leftMotor.getDeviceTemp();

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50, 
                                                                              leftVoltage, 
                                                                              leftSupplyCurrent, 
                                                                              leftTorqueCurrent, 
                                                                              leftTemperature));

    tryUntilOk(5, () -> leftMotor.optimizeBusUtilization(0, 1));

    rightVoltage = rightMotor.getMotorVoltage();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();
    rightTorqueCurrent = rightMotor.getTorqueCurrent();
    rightTemperature = rightMotor.getDeviceTemp();

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50, 
                                                                              rightVoltage, 
                                                                              rightSupplyCurrent, 
                                                                              rightTorqueCurrent, 
                                                                              rightTemperature));

    tryUntilOk(5, () -> rightMotor.optimizeBusUtilization(0, 1));

    bus
      .register(rightVoltage)
      .register(rightSupplyCurrent)
      .register(rightTorqueCurrent)
      .register(rightTemperature)
      .register(leftVoltage)
      .register(leftSupplyCurrent)
      .register(leftTorqueCurrent)
      .register(leftTemperature);

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      rightVoltage,
      rightSupplyCurrent,
      rightTorqueCurrent,
      rightTemperature,
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
    Logger.recordOutput("Elevator/Winch/TargetPosition", position);
  }

  @Override
  public void updateInputs(WinchInputs inputs) {
    inputs.left.connected = leftMotor.isConnected();
    inputs.left.position = rotationToDistance(leftMotor.getPosition().getValue());
    inputs.left.velocity = rotationToLinearVelocity(leftMotor.getVelocity().getValue());
    inputs.left.appliedVoltage = leftVoltage.getValue();
    inputs.left.supplyCurrent = leftSupplyCurrent.getValue();
    inputs.left.torqueCurrent = leftTorqueCurrent.getValue();
    inputs.left.temperature = leftTemperature.getValue();

    inputs.right.connected = rightMotor.isConnected();
    inputs.right.position = rotationToDistance(rightMotor.getPosition().getValue());
    inputs.right.velocity = rotationToLinearVelocity(rightMotor.getVelocity().getValue());
    inputs.right.appliedVoltage = rightVoltage.getValue();
    inputs.right.supplyCurrent = rightSupplyCurrent.getValue();
    inputs.right.torqueCurrent = rightTorqueCurrent.getValue();
    inputs.right.temperature = rightTemperature.getValue();

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
   * Converts rotational velocity (rotations per second) to linear velocity (meters per second)
   *
   * @param velocity Rotational velocity
   * @return Linear velocity
   */
  public LinearVelocity rotationToLinearVelocity(AngularVelocity velocity) {
    return Units.MetersPerSecond.of((2 * Math.PI * WHEEL_RADIUS.in(Units.Meter) * velocity.in(Units.RotationsPerSecond)) / GEARING);
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
            .withKI(ElevatorConstants.PID_SCORE.i.get())
            .withKD(ElevatorConstants.PID_CLIMB.d.get())
            .withKS(ElevatorConstants.PID_CLIMB.s.get())
            .withKG(ElevatorConstants.PID_CLIMB.g.get())
            .withKA(ElevatorConstants.PID_SCORE.a.get())
            .withKV(ElevatorConstants.PID_CLIMB.v.get())
            .withGravityType(GravityTypeValue.Elevator_Static);
    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT);
    krakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(krakenConfig));
    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(krakenConfig));
    tryUntilOk(5, () -> rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false)));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leftMotor.setVoltage(voltage.in(Units.Volts));
  }

}

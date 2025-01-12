package frc.robot.subsystems.elevator.winch;

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
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.Constants;
import lombok.Getter;

public class WinchIOKraken implements WinchIO {

  private static final double GEARING = 12;
  private static final Distance WHEEL_RADIUS = Units.Inches.of(1);

  @Getter private TalonFX leftMotor;
  @Getter private TalonFX rightMotor;

  /**
   * Contructor for real winch with Krakens
   *
   * @param leftMotor
   * @param rightMotor
   */
  public WinchIOKraken(TalonFX leftMotor, TalonFX rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
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
  }

  @Override
  public void setPosition(Distance position) {

    double p = meterToRotation(position);

    PositionTorqueCurrentFOC leftRequest = new PositionTorqueCurrentFOC(p).withSlot(0);
    leftMotor.setControl(leftRequest);
  }

  @Override
  public void updateInputs(WinchInputs inputs) {
    inputs.left.updateInputs(leftMotor);
    inputs.right.updateInputs(rightMotor);
    inputs.position = Units.Meter.of(rotationToMeter(leftMotor.getPosition().getValue()));
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
  public double rotationToMeter(Angle a) {
    return (2 * Math.PI * WHEEL_RADIUS.in(Units.Meter) * a.in(Units.Rotation)) / GEARING;
  }
}

package frc.robot.subsystems.coral_intake.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import lombok.Setter;

import org.littletonrobotics.junction.Logger;

public class PivotIONeo implements PivotIO {

  private SparkMax pivotMotor;
  private DutyCycleEncoder encoder;

  private Angle zeroedAngle = Degrees.zero();

  /** Constructor for pivotIO for Neo motors. */
  public PivotIONeo(int pivotMotorID, int encoderID) {
    encoder = new DutyCycleEncoder(encoderID);

    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.smartCurrentLimit(60);
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.inverted(true);
    pivotConfig.encoder.positionConversionFactor(1.0 / 50);
    pivotConfig.encoder.velocityConversionFactor(1.0 / 50 / 60);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotMotor.getEncoder().setPosition(getAbsolutePosition().in(Rotations));
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.position = Rotations.of(pivotMotor.getEncoder().getPosition());
    inputs.absolutePosition = getAbsolutePosition();
    inputs.encoderPosition = getAbsoluteEncoderPosition();
    inputs.velocity = Units.RPM.of(pivotMotor.getEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
    inputs.supplyCurrent = Units.Amps.of(pivotMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(pivotMotor.getMotorTemperature());

    Logger.recordOutput("CoralIntake/ZeroedAbsoluteEncoder", getZeroedAbsoluteEncoderPosition());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    pivotMotor.setVoltage(voltage);
  }

  /**
   * Gets the real absolute position of the coral intake pivot arm
   */
  private Angle getAbsolutePosition() {
    return CoralIntakeConstants.coralIntakeMaxAngle.minus(getZeroedAbsoluteEncoderPosition().times(0.5));
  }

  /**
   * Gets the zeroed absolute encoder position
   */
  private Angle getZeroedAbsoluteEncoderPosition() {
    double rads = getAbsoluteEncoderPosition().minus(CoralIntakeConstants.absoluteEncoderOffset).minus(zeroedAngle).in(Radians);

    rads = (rads + 0.5 + 2 * Math.PI) % (2 * Math.PI) - 0.5;

    return Radians.of(rads);
  }

  /**
   * Gets the raw absolute encoder position in rotations
   */
  private Angle getAbsoluteEncoderPosition() {
    return Rotations.of(encoder.get());
  }

  /**
   * Zeros the pivot angle to the specified position.
   * @param position
   */
  public void zeroPivotAngle(Angle position) {
    zeroedAngle = getZeroedAbsoluteEncoderPosition().minus(position);
  }
}

package frc.robot.subsystems.coral_intake.pivot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.utils.Constants;
import frc.robot.utils.UnitUtil;

public class PivotIONeo implements PivotIO {

  private SparkMax pivotMotor;
  private DutyCycleEncoder encoder;

  /** Constructor for pivotIO for Neo motors. */
  public PivotIONeo(int pivotMotorID, int encoderID) {
    encoder = new DutyCycleEncoder(encoderID);

    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.smartCurrentLimit(30);
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.closedLoop.pidf(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(), CoralIntakeConstants.PID.d.get(), 0);
    pivotConfig.closedLoop.maxMotion.maxVelocity(0.5);
    pivotConfig.closedLoop.maxMotion.maxAcceleration(0.5);
    pivotConfig.inverted(true);
    pivotConfig.encoder.positionConversionFactor(1.0 / 50);
    pivotConfig.encoder.velocityConversionFactor(1.0 / 50 / 60);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  double previousDesiredVoltage = 0;

  @Override
  public void updateInputs(PivotInputs inputs) {
    Angle absolutePosition = Rotations.of(encoder.get());

    if (absolutePosition.lt(CoralIntakeConstants.breakPoint)) {
      absolutePosition = absolutePosition.plus(Rotations.one());
    }

    inputs.position = Rotations.of(pivotMotor.getEncoder().getPosition());
    inputs.absolutePosition = absolutePosition.times(-0.5).plus(CoralIntakeConstants.zeroOffset);
    inputs.encoderPosition = Rotations.of(encoder.get());
    inputs.velocity = Units.RPM.of(pivotMotor.getEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
    inputs.supplyCurrent = Units.Amps.of(pivotMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(pivotMotor.getMotorTemperature());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    pivotMotor.setVoltage(voltage);
  }
}

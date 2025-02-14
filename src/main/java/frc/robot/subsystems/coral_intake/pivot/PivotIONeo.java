package frc.robot.subsystems.coral_intake.pivot;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.utils.Constants;

public class PivotIONeo implements PivotIO {

  private SparkMax pivotMotor;
  private DutyCycleEncoder encoder;

  private Angle positionOffset;
  private Angle seedPosition;

  // Setup for PID, FF, and SVG

  private ArmFeedforward pivotFeedforward;

  private TrapezoidProfile.State currentPivotState = new TrapezoidProfile.State();
  private TrapezoidProfile.State targetPivotState = new TrapezoidProfile.State();
  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(CoralIntakeConstants.PID.maxPivotV.get(), CoralIntakeConstants.PID.maxPivotA.get()));

  /** Constructor for pivotIO for Neo motors. */
  public PivotIONeo(int pivotMotorID, int encoderID) {
    encoder = new DutyCycleEncoder(encoderID);
    seedPosition = Units.Rotations.of(encoder.get() / 2);
    positionOffset = seedPosition.plus(Rotations.of(CoralIntakeConstants.pivotOffset.get()));

    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.smartCurrentLimit((int) Constants.NEO_CURRENT_LIMIT.in(Units.Amps));
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.closedLoop.pidf(0,0,0,0);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotFeedforward = new ArmFeedforward(0, 0, 0);
    currentPivotState.position = getPosition().in(Units.Degrees);
  }

  @Override
  public void setPosition(Angle position) {
    targetPivotState = new TrapezoidProfile.State(position.in(Units.Rotations), 0);
    currentPivotState = trapezoidProfile.calculate(0.02, currentPivotState, targetPivotState);
    pivotMotor.getClosedLoopController().setReference(currentPivotState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, pivotFeedforward.calculate(currentPivotState.position, currentPivotState.velocity));
  }

  private Angle getPosition() {
    return Units.Rotations.of(pivotMotor.getEncoder().getPosition() * 50).plus(positionOffset);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.position = getPosition();
    inputs.velocity = Units.RPM.of(pivotMotor.getEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(pivotMotor.getBusVoltage());
    inputs.supplyCurrent = Units.Amps.of(pivotMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(pivotMotor.getMotorTemperature());

    updateConstants();
  }

  private void updateConstants() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> {
        pivotFeedforward = new ArmFeedforward(CoralIntakeConstants.PID.s.get(), CoralIntakeConstants.PID.g.get(), CoralIntakeConstants.PID.v.get());
    }, CoralIntakeConstants.PID.s, CoralIntakeConstants.PID.v, CoralIntakeConstants.PID.g);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      SparkMaxConfig motorConfig = new SparkMaxConfig();

      motorConfig.apply(
        new EncoderConfig()
          .positionConversionFactor(1.0/25)
          .velocityConversionFactor(1.0/25/60)
      ).apply(
        new ClosedLoopConfig()
          .pid(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(), CoralIntakeConstants.PID.d.get())
      );

      pivotMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }, CoralIntakeConstants.PID.p, CoralIntakeConstants.PID.i, CoralIntakeConstants.PID.d);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(CoralIntakeConstants.PID.maxPivotV.get(), CoralIntakeConstants.PID.maxPivotA.get()));
    }, CoralIntakeConstants.PID.maxPivotA, CoralIntakeConstants.PID.maxPivotV);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      positionOffset = seedPosition.plus(Rotations.of(CoralIntakeConstants.pivotOffset.get()));
    }, CoralIntakeConstants.pivotOffset);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    pivotMotor.setVoltage(voltage);
  }
}

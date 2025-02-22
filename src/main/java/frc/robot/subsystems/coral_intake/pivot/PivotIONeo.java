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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.utils.Constants;
import frc.robot.utils.UnitUtil;

public class PivotIONeo implements PivotIO {

  private SparkMax pivotMotor;
  private DutyCycleEncoder encoder;
  private PIDController pid = new PIDController(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(), CoralIntakeConstants.PID.d.get());

  /** Constructor for pivotIO for Neo motors. */
  public PivotIONeo(int pivotMotorID, int encoderID) {
    encoder = new DutyCycleEncoder(encoderID);

    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.smartCurrentLimit((int) Constants.NEO_CURRENT_LIMIT.in(Units.Amps));
    pivotConfig.idleMode(IdleMode.kCoast);
    pivotConfig.closedLoop.pidf(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(), CoralIntakeConstants.PID.d.get(), 0);
    pivotConfig.inverted(true);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setPositionWithFeedforward(Angle desired, Angle current,  Voltage ff) {
    pivotMotor.setVoltage(pid.calculate(current.baseUnitMagnitude(), desired.baseUnitMagnitude()) + ff.in(Volts));

    // pivotMotor.getClosedLoopController().setReference(position.in(Rotations), ControlType.kPosition,
    //     ClosedLoopSlot.kSlot0,
    //     ff.in(Volts));
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.position = Rotations.of(pivotMotor.getEncoder().getPosition());
    inputs.absolutePosition = Rotations.of(encoder.get());
    inputs.velocity = Units.RPM.of(pivotMotor.getEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
    inputs.supplyCurrent = Units.Amps.of(pivotMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(pivotMotor.getMotorTemperature());

    updateConstants();
  }

  private void updateConstants() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      SparkMaxConfig motorConfig = new SparkMaxConfig();

      pid = new PIDController(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(), CoralIntakeConstants.PID.d.get());

      // motorConfig.apply(
      //     new EncoderConfig()
      //         .positionConversionFactor(1.0 / 50)
      //         .velocityConversionFactor(1.0 / 50 / 60))
      //     .apply(
      //         new ClosedLoopConfig()
      //             .pid(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(),
      //                 CoralIntakeConstants.PID.d.get()));

      // motorConfig.closedLoop.maxMotion.maxAcceleration(CoralIntakeConstants.PID.maxPivotA.get());
      // motorConfig.closedLoop.maxMotion.maxVelocity(CoralIntakeConstants.PID.maxPivotV.get());


      // pivotMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }, CoralIntakeConstants.PID.p, CoralIntakeConstants.PID.i, CoralIntakeConstants.PID.d, CoralIntakeConstants.PID.maxPivotA, CoralIntakeConstants.PID.maxPivotV);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    // pivotMotor.setVoltage(voltage);
  }
}

package frc.robot.subsystems.coral_intake.pivot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;

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

  // Setup for PID, FF, and SVG

  private ArmFeedforward pivotFeedforward;

  private TrapezoidProfile.State currentPivotState = new TrapezoidProfile.State();
  private TrapezoidProfile.State targetPivotState = new TrapezoidProfile.State();
  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      CoralIntakeConstants.PID.maxPivotV.get(), CoralIntakeConstants.PID.maxPivotA.get()));

  /** Constructor for pivotIO for Neo motors. */
  public PivotIONeo(int pivotMotorID, int encoderID) {
    encoder = new DutyCycleEncoder(encoderID);

    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.smartCurrentLimit((int) Constants.NEO_CURRENT_LIMIT.in(Units.Amps));
    pivotConfig.idleMode(IdleMode.kCoast);
    pivotConfig.closedLoop.pidf(0, 0, 0, 0);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotFeedforward = new ArmFeedforward(0, 0, 0);
    currentPivotState.position = getPosition().in(Units.Radians);
  }

  @Override
  public void setPosition(Angle position) {
    targetPivotState = new TrapezoidProfile.State(position.in(Units.Rotations), 0);
    TrapezoidProfile.State realCurrentState = new TrapezoidProfile.State(getPosition().in(Units.Rotations), pivotMotor.getEncoder().getVelocity());
    currentPivotState = trapezoidProfile.calculate(0.02, realCurrentState, targetPivotState);

    double deltaIntakePosition = position.minus(getPosition()).in(Rotations);
    double deltaMotorPosition = -deltaIntakePosition * 50;
    double desiredMotorPosition = pivotMotor.getEncoder().getPosition() + deltaMotorPosition;

    Logger.recordOutput("CoralIntake/Uhh/givenPosition", position);
    Logger.recordOutput("CoralIntake/Uhh/deltaIntakePosition", deltaIntakePosition);
    Logger.recordOutput("CoralIntake/Uhh/deltaMotorPosition", deltaMotorPosition);
    Logger.recordOutput("CoralIntake/Uhh/desiredMotorPosition", desiredMotorPosition);
    Logger.recordOutput("CoralIntake/Uhh/ff", pivotFeedforward.calculate(currentPivotState.position, currentPivotState.velocity));
    
    // pivotMotor.getClosedLoopController().setReference(desiredMotorPosition, ControlType.kPosition,
    //     ClosedLoopSlot.kSlot0,
    //     pivotFeedforward.calculate(currentPivotState.position, currentPivotState.velocity));
  }

  private Angle getPosition() {
    Angle continuousEncoderPosition = UnitUtil
        .normalize(Units.Rotations.of(encoder.get()).minus(CoralIntakeConstants.breakPoint))
        .plus(CoralIntakeConstants.breakPoint);
    return continuousEncoderPosition.times(-0.5).plus(CoralIntakeConstants.zeroOffset);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.position = Rotations.of(pivotMotor.getEncoder().getPosition());
    inputs.absolutePosition = Rotations.of(encoder.get());
    inputs.velocity = Units.RPM.of(pivotMotor.getEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(pivotMotor.getBusVoltage());
    inputs.supplyCurrent = Units.Amps.of(pivotMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(pivotMotor.getMotorTemperature());

    updateConstants();

    Logger.recordOutput("CoralIntake/AngleTuning/g_position", getPosition().in(Degree));
  }

  private void updateConstants() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      pivotFeedforward = new ArmFeedforward(CoralIntakeConstants.PID.s.get(), CoralIntakeConstants.PID.g.get(),
          CoralIntakeConstants.PID.v.get());
    }, CoralIntakeConstants.PID.s, CoralIntakeConstants.PID.v, CoralIntakeConstants.PID.g);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      SparkMaxConfig motorConfig = new SparkMaxConfig();

      motorConfig.apply(
          new EncoderConfig()
              .positionConversionFactor(1.0 / 25)
              .velocityConversionFactor(1.0 / 25 / 60))
          .apply(
              new ClosedLoopConfig()
                  .pid(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(),
                      CoralIntakeConstants.PID.d.get()));

      pivotMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }, CoralIntakeConstants.PID.p, CoralIntakeConstants.PID.i, CoralIntakeConstants.PID.d);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(CoralIntakeConstants.PID.maxPivotV.get(),
          CoralIntakeConstants.PID.maxPivotA.get()));
    }, CoralIntakeConstants.PID.maxPivotA, CoralIntakeConstants.PID.maxPivotV);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    // pivotMotor.setVoltage(voltage);
  }
}

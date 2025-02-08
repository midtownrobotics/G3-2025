package frc.robot.subsystems.coral_intake.pivot;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.utils.Constants;
import lombok.Getter;

public class PivotIONeo implements PivotIO {

  private @Getter SparkMax pivotMotor;
  private @Getter DutyCycleEncoder encoder;

  // Setup for PID, FF, and SVG

  private ArmFeedforward pivotFeedforward;

  private TrapezoidProfile.State currentPivotState = new TrapezoidProfile.State();
  private TrapezoidProfile.State targetPivotState = new TrapezoidProfile.State();
  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(CoralIntakeConstants.maxPivotV.get(), CoralIntakeConstants.maxPivotA.get()));

  /** Constructor for pivotIO for Neo motors. */
  public PivotIONeo(int pivotMotorID, int ecoderID) {
    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.smartCurrentLimit((int) Constants.NEO_CURRENT_LIMIT.in(Units.Amps));
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.closedLoop.pidf(0,0,0,0);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

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
    return Units.Rotations.of(pivotMotor.getAbsoluteEncoder().getPosition());
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.position = getPosition();
    inputs.velocity = Units.RPM.of(pivotMotor.getAbsoluteEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(pivotMotor.getBusVoltage());
    inputs.supplyCurrent = Units.Amps.of(pivotMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(pivotMotor.getMotorTemperature());
    inputs.offsetedPosition = inputs.absolutePosition.plus(CoralIntakeConstants.pivotOffset);


    updateConstants();
  }

  private void updateConstants() {
    LoggedTunableNumber.ifChanged(hashCode(), () -> {
        pivotFeedforward = new ArmFeedforward(CoralIntakeConstants.coralIntakeKs.get(), CoralIntakeConstants.coralIntakeKg.get(), CoralIntakeConstants.coralIntakeKv.get());
    }, CoralIntakeConstants.coralIntakeKg, CoralIntakeConstants.coralIntakeKs, CoralIntakeConstants.coralIntakeKv);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      SparkMaxConfig motorConfig = new SparkMaxConfig();
      
      motorConfig.apply(
        new EncoderConfig()
          .positionConversionFactor(1.0/25)
          .velocityConversionFactor(1.0/25/60)
      ).apply(
        new ClosedLoopConfig()
          .pid(CoralIntakeConstants.coralIntakeP.get(), 0, CoralIntakeConstants.coralIntakeD.get())
      );

      System.out.println("PID");

      pivotMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }, CoralIntakeConstants.coralIntakeP, CoralIntakeConstants.coralIntakeD);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(CoralIntakeConstants.maxPivotV.get(), CoralIntakeConstants.maxPivotA.get()));
    }, CoralIntakeConstants.maxPivotA, CoralIntakeConstants.maxPivotV);
  }
}

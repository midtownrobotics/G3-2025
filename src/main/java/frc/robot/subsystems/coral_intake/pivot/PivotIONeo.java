package frc.robot.subsystems.coral_intake.pivot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.Constants;
import lombok.Getter;

public class PivotIONeo implements PivotIO {

  private @Getter SparkMax pivotMotor;
  private @Getter DutyCycleEncoder encoder;
  private @Getter PIDController pivotPID;


  /** Constructor for pivotIO for Neo motors. */
  public PivotIONeo(int pivotMotorID, int ecoderID) {
    pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
    pivotMotor.configure(new SparkMaxConfig().smartCurrentLimit((int) Constants.NEO_CURRENT_LIMIT.in(Units.Amps))
    .idleMode(IdleMode.kBrake),

    ResetMode.kResetSafeParameters,
    PersistMode.kNoPersistParameters);

    pivotPID = new PIDController(0, 0, 0);
  }



  @Override
  public void setPosition(Angle position) {
    pivotMotor.setVoltage(pivotPID.calculate(encoder.get(), position.in(Units.Rotations)));
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.position = Units.Rotations.of(pivotMotor.getAbsoluteEncoder().getPosition());
    inputs.velocity = Units.RPM.of(pivotMotor.getAbsoluteEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(pivotMotor.getBusVoltage());
    inputs.supplyCurrent = Units.Amps.of(pivotMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(pivotMotor.getMotorTemperature());
  }
}

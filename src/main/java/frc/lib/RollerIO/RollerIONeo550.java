package frc.lib.RollerIO;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.Constants;
import lombok.Getter;

public class RollerIONeo550 implements RollerIO {
 private @Getter SparkMax motor;

  /** Constructor for rollerIO for Neo motors. */
  public RollerIONeo550(int motorID, IdleMode idleMode) {
    motor = new SparkMax(motorID, MotorType.kBrushless);
    motor.configure(new SparkMaxConfig().smartCurrentLimit((int) Constants.NEO_550_CURRENT_LIMIT.in(Units.Amps))
    .idleMode(idleMode),

    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setOutput(double output) {
      motor.set(output);
  }


  @Override
  public void updateInputs(RollerInputs inputs) {
    inputs.position = Units.Rotations.of(motor.getEncoder().getPosition());
    inputs.velocity = Units.RPM.of(motor.getEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(motor.getBusVoltage()*motor.getAppliedOutput());
    inputs.supplyCurrent = Units.Amps.of(motor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(motor.getMotorTemperature());
  }
}

package frc.robot.subsystems.coral_intake.belt;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.Constants;
import lombok.Getter;

public class BeltIONeo implements BeltIO {

  private @Getter SparkMax beltMotor;

  /** Constructor for beltIO for Neo motors. */
  public BeltIONeo(int beltMotorID) {
    beltMotor = new SparkMax(beltMotorID, MotorType.kBrushless);
    beltMotor.configure(new SparkMaxConfig().smartCurrentLimit((int) Constants.NEO_CURRENT_LIMIT.in(Units.Amps))
    .idleMode(IdleMode.kBrake),

    ResetMode.kResetSafeParameters,
    PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    beltMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(BeltInputs inputs) {
    inputs.position = Units.Rotations.of(beltMotor.getAbsoluteEncoder().getPosition());
    inputs.velocity = Units.RPM.of(beltMotor.getAbsoluteEncoder().getVelocity());
    inputs.appliedVoltage = Units.Volts.of(beltMotor.getBusVoltage());
    inputs.supplyCurrent = Units.Amps.of(beltMotor.getOutputCurrent());
    inputs.temperature = Units.Celsius.of(beltMotor.getMotorTemperature());
  }
}


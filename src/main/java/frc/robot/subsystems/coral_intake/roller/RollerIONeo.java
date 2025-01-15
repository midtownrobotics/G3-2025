package frc.robot.subsystems.coral_intake.roller;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import frc.robot.utils.Constants;
import lombok.Getter;

public class RollerIONeo implements RollerIO {
 private @Getter SparkMax beltMotor;

  /** Constructor for rollerIO for Neo motors. */
  public RollerIONeo(int beltMotorID) {
    beltMotor = new SparkMax(beltMotorID, MotorType.kBrushless);
    beltMotor.configure(new SparkMaxConfig().smartCurrentLimit((int) Constants.NEO_CURRENT_LIMIT.in(Units.Amps))
    .idleMode(IdleMode.kBrake),

    ResetMode.kResetSafeParameters,
    PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVoltage(int voltage) {
    beltMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }
}

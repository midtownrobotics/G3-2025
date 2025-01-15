package frc.robot.subsystems.algae_claw.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.Constants;
import lombok.Getter;

public class RollerIOKraken implements RollerIO {

  @Getter private TalonFX topRoller;
  @Getter private TalonFX bottomRoller;

  /** Constructor for rollerIO for kraken motors. */
  public RollerIOKraken(int topRollerID, int bottomRollerID) {
    topRoller = new TalonFX(topRollerID);
    bottomRoller = new TalonFX(bottomRollerID);
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();

    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(Constants.KRAKEN_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(1);

    topRoller.getConfigurator().apply(krakenConfig);
    bottomRoller.getConfigurator().apply(krakenConfig);
  }


  @Override
  public void setVoltage(int voltage) {
    topRoller.setVoltage(voltage);
    bottomRoller.setVoltage(voltage);
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }
}

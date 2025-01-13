package frc.robot.subsystems.algae_claw.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.utils.Constants;
import lombok.Getter;

public class RollerIOKraken implements RollerIO {

  @Getter private TalonFX topRoller;
  @Getter private TalonFX bottomRoller;

  public RollerIOKraken(int topRollerID, int bottomRollerID) {
    topRoller = new TalonFX(topRollerID);
    bottomRoller = new TalonFX(bottomRollerID);
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    krakenConfig.Slot0 =
        new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKG(0)
            .withKA(0)
            .withKV(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(40)
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

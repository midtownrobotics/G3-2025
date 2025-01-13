package frc.robot.subsystems.algae_claw.wrist;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.utils.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import lombok.Getter;

public class WristIOKraken implements WristIO {

  @Getter private TalonFX wristMotor;
  @Getter private DutyCycleEncoder encoder;
  @Getter private PIDController wristPID;

  public WristIOKraken(int wristMotorID, int encoderID) {
    wristMotor = new TalonFX(wristMotorID);
    encoder = new DutyCycleEncoder(new DigitalInput(encoderID));

    wristPID = new PIDController(0, 0, 0);

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

    wristMotor.getConfigurator().apply(krakenConfig);
  }

  @Override
  public void setPosition(Angle position) {
    wristMotor.setVoltage(wristPID.calculate(encoder.get(), position.in(Units.Rotations)));
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }
}

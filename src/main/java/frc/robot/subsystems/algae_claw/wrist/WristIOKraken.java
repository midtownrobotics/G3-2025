package frc.robot.subsystems.algae_claw.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.Constants;
import lombok.Getter;

public class WristIOKraken implements WristIO {

  @Getter private TalonFX wristMotor;
  @Getter private DutyCycleEncoder encoder;
  @Getter private PIDController wristPID;

  @Getter private StatusSignal<Angle> position;
  @Getter private StatusSignal<AngularVelocity> velocity;
  @Getter private StatusSignal<Voltage> voltage;
  @Getter private StatusSignal<Current> supplyCurrent;
  @Getter private StatusSignal<Current> torqueCurrent;
  @Getter private StatusSignal<Temperature> temperature;

  /** Constructor for wristIO for kraken motors. */
  public WristIOKraken(int wristMotorID, int encoderID) {
    wristMotor = new TalonFX(wristMotorID);
    encoder = new DutyCycleEncoder(new DigitalInput(encoderID));

    wristPID = new PIDController(0, 0, 0);

    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();

    krakenConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Constants.KRAKEN_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(Constants.KRAKEN_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(1);

    wristMotor.getConfigurator().apply(krakenConfig);

    position = wristMotor.getPosition();
    velocity = wristMotor.getVelocity();
    voltage = wristMotor.getMotorVoltage();
    supplyCurrent = wristMotor.getSupplyCurrent();
    torqueCurrent = wristMotor.getTorqueCurrent();
    temperature = wristMotor.getDeviceTemp();

    position.setUpdateFrequency(50);
    velocity.setUpdateFrequency(50);
    voltage.setUpdateFrequency(50);
    supplyCurrent.setUpdateFrequency(50);
    torqueCurrent.setUpdateFrequency(50);
    temperature.setUpdateFrequency(50);

    wristMotor.optimizeBusUtilization();
  }

  @Override
  public void setPosition(Angle position) {
    wristMotor.setVoltage(wristPID.calculate(encoder.get(), position.in(Units.Rotations)));
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    inputs.connected = wristMotor.isConnected();
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = voltage.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();
  }
}

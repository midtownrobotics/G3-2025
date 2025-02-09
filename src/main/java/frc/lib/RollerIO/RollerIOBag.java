package frc.lib.RollerIO;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.Constants;

public class RollerIOBag implements RollerIO {

    private SparkMax motor;

    /**
     * Constructor for Bag Roller
     */
    public RollerIOBag(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushed);

        SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig.smartCurrentLimit((int) Constants.BAG_CURRENT_LIMIT.in(Units.Amps));
        motor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(RollerInputs inputs) {
        inputs.appliedVoltage = Units.Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.velocity = Units.RPM.of(-1);
        inputs.temperature = Units.Kelvin.of(-1);
        inputs.supplyCurrent = Units.Amps.of(motor.getOutputCurrent());
        inputs.position = Units.Rotations.of(-1);

    }}

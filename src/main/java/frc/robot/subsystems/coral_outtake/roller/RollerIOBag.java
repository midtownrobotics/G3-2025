package frc.robot.subsystems.coral_outtake.roller;


import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.Constants;

public class RollerIOBag implements RollerIO {

    private SparkMax rollerMotor;

    /**
     * Constructor for Bag Roller
     */
    public RollerIOBag(int rollerMotorID) {
        rollerMotor = new SparkMax(rollerMotorID, MotorType.kBrushed);

        SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig.smartCurrentLimit((int) Constants.BAG_CURRENT_LIMIT.in(Units.Amps));
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(int voltage) {
        rollerMotor.setVoltage(voltage);
    }

    public Voltage getVoltage() {
        return Voltage.ofBaseUnits(rollerMotor.getAppliedOutput()*rollerMotor.getBusVoltage(), Volts);
    }

    @Override
    public void updateInputs(RollerInputs inputs) {
        inputs.appliedVoltage = Units.Volts.of(rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage());
        inputs.velocity = Units.RPM.of(rollerMotor.getEncoder().getVelocity());
        inputs.temperature = Units.Celsius.of(rollerMotor.getMotorTemperature());
        inputs.supplyCurrent = Units.Amps.of(rollerMotor.getOutputCurrent());
        inputs.position = Units.Rotations.of(rollerMotor.getEncoder().getPosition());

    }}

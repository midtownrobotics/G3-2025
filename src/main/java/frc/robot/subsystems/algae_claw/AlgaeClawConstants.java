package frc.robot.subsystems.algae_claw;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class AlgaeClawConstants {
    public static final Voltage INTAKE_ROLLER_VOLTAGE = Volts.of(7); // TODO
    public static final Voltage SHOOT_ROLLER_VOLTAGE = Volts.of(-7); // TODO
    public static final Voltage HOLD_PIECE_ROLLER_VOLTAGE = Volts.of(0.5); // TODO
    public static final Current CURRENT_SHOOT_MINIMUM_DETECTION = Units.Amps.of(5); // TODO
    public static final Current CURRENT_INTAKE_MAXIMUM_DETECTION = Units.Amps.of(5);// TODO
}

package frc.robot.subsystems.algae_claw;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.LoggedTunableNumber;

public class AlgaeClawConstants {
    public static final Voltage INTAKE_ROLLER_VOLTAGE = Volts.of(7); // TODO
    public static final Voltage SHOOT_ROLLER_VOLTAGE = Volts.of(-7); // TODO
    public static final Voltage HOLD_PIECE_ROLLER_VOLTAGE = Volts.of(0.5); // TODO
    public static final Current CURRENT_SHOOT_MINIMUM_DETECTION = Units.Amps.of(5); // TODO
    public static final Current CURRENT_INTAKE_MAXIMUM_DETECTION = Units.Amps.of(5);// TODO
    public static final AngularVelocity MIN_ANGULAR_VELOCITY_PIECE_DETECTION = Units.RotationsPerSecond.of(0); // TODO

    public class PID {
        public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_CLIMB/p", 0.0);
        public static final LoggedTunableNumber i = new LoggedTunableNumber("Elevator/PID_CLIMB/i", 0.0);
        public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_CLIMB/d", 0.0);
        public static final LoggedTunableNumber s = new LoggedTunableNumber("Elevator/PID_CLIMB/s", 0.0);
        public static final LoggedTunableNumber g = new LoggedTunableNumber("Elevator/PID_CLIMB/g", 0.0);
        public static final LoggedTunableNumber v = new LoggedTunableNumber("Elevator/PID_CLIMB/v", 0.0);
        public static final LoggedTunableNumber a = new LoggedTunableNumber("Elevator/PID_CLIMB/a", 0.0);
    }
}

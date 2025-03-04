package frc.robot.subsystems.algae_claw;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.dashboard.LoggedTunableNumber;

public class AlgaeClawConstants {
    public static final Angle absolutePositionOffset = Radians.of(0);

    public static final Voltage INTAKE_ROLLER_VOLTAGE = Volts.of(7); // TODO
    public static final Voltage SHOOT_ROLLER_VOLTAGE = Volts.of(-7); // TODO
    public static final Voltage HOLD_PIECE_ROLLER_VOLTAGE = Volts.of(0.5); // TODO
    public static final Current CURRENT_SHOOT_MINIMUM_DETECTION = Units.Amps.of(5); // TODO
    public static final Current CURRENT_INTAKE_MAXIMUM_DETECTION = Units.Amps.of(5);// TODO
    public static final AngularVelocity MIN_ANGULAR_VELOCITY_PIECE_DETECTION = Units.RotationsPerSecond.of(0); // TODO

    public static final Angle algaeClawMaxAngle = Degrees.of(90);
    public static final Angle algaeClawMinAngle = Degrees.of(0);

    public class PID {
        public static final LoggedTunableNumber p = new LoggedTunableNumber("AlgaeClaw/PID/p", 0.0);
        public static final LoggedTunableNumber i = new LoggedTunableNumber("AlgaeClaw/PID/i", 0.0);
        public static final LoggedTunableNumber d = new LoggedTunableNumber("AlgaeClaw/PID/d", 0.0);
        public static final LoggedTunableNumber s = new LoggedTunableNumber("AlgaeClaw/PID/s", 0.0);
        public static final LoggedTunableNumber g = new LoggedTunableNumber("AlgaeClaw/PID/g", 0.0);
        public static final LoggedTunableNumber v = new LoggedTunableNumber("AlgaeClaw/PID/v", 0.0);
        public static final LoggedTunableNumber a = new LoggedTunableNumber("AlgaeClaw/PID/a", 0.0);
    }
}

package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.lib.dashboard.LoggedTunableNumber;

public class CoralIntakeConstants {
    public static final Time coralDetectionIdleDelay = Milliseconds.of(100);

    public static final Angle coralIntakeMaxAngle = Degrees.of(135);
    public static final Angle coralIntakeMinAngle = Degrees.of(-14);

    public static final Angle absoluteEncoderOffset = Radians.of(5.573);

    public final class PID {
        public static final LoggedTunableNumber s = new LoggedTunableNumber("CoralIntake/s", 0.18);
        public static final LoggedTunableNumber v = new LoggedTunableNumber("CoralIntake/v", 0.55);
        public static final LoggedTunableNumber g = new LoggedTunableNumber("CoralIntake/g", 0.54);
        public static final LoggedTunableNumber p = new LoggedTunableNumber("CoralIntake/p", 5.7);
        public static final LoggedTunableNumber i = new LoggedTunableNumber("CoralIntake/i", 0.15);
        public static final LoggedTunableNumber d = new LoggedTunableNumber("CoralIntake/d", 0.2);

        public static final LoggedTunableNumber maxPivotV = new LoggedTunableNumber("CoralIntake/maxV", 12);
        public static final LoggedTunableNumber maxPivotA = new LoggedTunableNumber("CoralIntake/maxA", 33);
    }
}

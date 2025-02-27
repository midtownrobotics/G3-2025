package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.utils.UnitUtil;

public class CoralIntakeConstants {
    public static final Time coralDetectionIdleDelay = Milliseconds.of(100);

    public static final Angle tuningAngleIncrementDecrementAmmount = Degrees.of(5);

    // TODO: Find out max and min angles
    /** *** In radians *** */
    public static final LoggedTunableNumber coralIntakeMaxAngle = new LoggedTunableNumber("CoralIntake/maxAngleRadians",
            Units.degreesToRadians(135));
    public static final Angle coralIntakeMinAngle = Degrees.of(-10);

    /** True angle of intake when all the way back */
    public static final Angle g_max = Degrees.of(141);
    /** True angle of intake when all the way forward */
    public static final Angle g_min = Degrees.of(-21.783);
    /** Encoder reading when arm is at max. Configurable */
    public static final Angle e_max = Radians.of(6.033);
    /** Encoder reading when arm is at min */
    public static final Angle e_min = UnitUtil.normalize(e_max.plus(g_max.minus(g_min).times(2)));
    /** Breakpoint */
    public static final Angle breakPoint = e_max.lt(e_min)
            ? UnitUtil.normalize((e_max.plus(e_min)).div(2).plus(Degrees.of(180)))
            : UnitUtil.normalize((e_max.plus(e_min)).div(2));
    /** The zero offset */
    public static final Angle zeroOffset = g_max.plus(e_max.times(0.5));

    public final class PID {
        public static final LoggedTunableNumber s = new LoggedTunableNumber("CoralIntake/s", 0.04);
        public static final LoggedTunableNumber v = new LoggedTunableNumber("CoralIntake/v", 0.8);
        public static final LoggedTunableNumber g = new LoggedTunableNumber("CoralIntake/g", 0.4);
        public static final LoggedTunableNumber p = new LoggedTunableNumber("CoralIntake/p", 3.7);
        public static final LoggedTunableNumber i = new LoggedTunableNumber("CoralIntake/i", 0);
        public static final LoggedTunableNumber d = new LoggedTunableNumber("CoralIntake/d", 0.2);

        public static final LoggedTunableNumber maxPivotV = new LoggedTunableNumber("CoralIntake/maxV", 8);
        public static final LoggedTunableNumber maxPivotA = new LoggedTunableNumber("CoralIntake/maxA", 8);

    }

    public static final LoggedTunableNumber pivotOffset = new LoggedTunableNumber(
            "CoralIntake/pivotAbsoluteEncoderOffset", 0.03);
}

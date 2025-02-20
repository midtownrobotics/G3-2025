package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.lib.LoggedTunableNumber;

public class CoralIntakeConstants {
    public static final Time coralDetectionIdleDelay = Milliseconds.of(100);

    public static final Angle tuningAngleIncrementDecrementAmmount = Degrees.of(5);

    // TODO: Find out max and min angles
    public static final Angle coralIntakeMaxAngle = Degrees.of(0);
    public static final Angle coralIntakeMinAngle = Degrees.of(0);

    public final class PID {
        public static final LoggedTunableNumber s = new LoggedTunableNumber("CoralIntake/s", 0.03);
        public static final LoggedTunableNumber v = new LoggedTunableNumber("CoralIntake/v", 0.23);
        public static final LoggedTunableNumber g = new LoggedTunableNumber("CoralIntake/g", 0.8);
        public static final LoggedTunableNumber p = new LoggedTunableNumber("CoralIntake/p", 5);
        public static final LoggedTunableNumber i = new LoggedTunableNumber("CoralIntake/i", 0);
        public static final LoggedTunableNumber d = new LoggedTunableNumber("CoralIntake/d", 0.5);

        public static final LoggedTunableNumber maxPivotV = new LoggedTunableNumber("CoralIntake/maxV", 3.5);
        public static final LoggedTunableNumber maxPivotA = new LoggedTunableNumber("CoralIntake/maxA", 10);

    }

    public static final LoggedTunableNumber pivotOffset = new LoggedTunableNumber("CoralIntake/pivotAbsoluteEncoderOffset", 0.03);
}

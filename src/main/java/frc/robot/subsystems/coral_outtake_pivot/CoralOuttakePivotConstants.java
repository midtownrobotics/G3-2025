package frc.robot.subsystems.coral_outtake_pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import frc.lib.dashboard.LoggedTunableNumber;

public class CoralOuttakePivotConstants {
    public static final Angle coralOuttakeMaxAngle = Degrees.of(141);
    public static final Angle coralOuttakeMinAngle = Degrees.of(-10);

    public static final Dimensionless coralOuttakePivotGearRatio = Value.of(50.0);

    public static final Angle absoluteEncoderOffset = Radians.of(2.458);

    public final class PID {
        public static final LoggedTunableNumber s = new LoggedTunableNumber("CoralOuttake/s", 0.0);
        public static final LoggedTunableNumber v = new LoggedTunableNumber("CoralOuttake/v", 0.0);
        public static final LoggedTunableNumber g = new LoggedTunableNumber("CoralOuttake/g", 0.0);
        public static final LoggedTunableNumber p = new LoggedTunableNumber("CoralOuttake/p", 0.0);
        public static final LoggedTunableNumber i = new LoggedTunableNumber("CoralOuttake/i", 0.0);
        public static final LoggedTunableNumber d = new LoggedTunableNumber("CoralOuttake/d", 0.0);

        public static final AngularVelocity maxPivotV = DegreesPerSecond.of(0);
        public static final AngularAcceleration maxPivotA = DegreesPerSecondPerSecond.of(0);
    }
}

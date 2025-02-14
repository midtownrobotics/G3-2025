package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.lib.LoggedTunableNumber;

public class CoralIntakeConstants {
    public static final Time coralDetectionIdleDelay = Milliseconds.of(100);

    // TODO: Find out max and min angles
    public static final Angle coralIntakeMaxAngle = Degrees.of(0);
    public static final Angle coralIntakeMinAngle = Degrees.of(0);

    public final class PID {
        public static final LoggedTunableNumber coralIntakeS = new LoggedTunableNumber("CoralIntake/coralIntakeS", 0.03);
        public static final LoggedTunableNumber coralIntakeV = new LoggedTunableNumber("CoralIntake/coralIntakeV", 0.23);
        public static final LoggedTunableNumber coralIntakeG = new LoggedTunableNumber("CoralIntake/coralIntakeG", 0.8);
        public static final LoggedTunableNumber coralIntakeP = new LoggedTunableNumber("CoralIntake/coralIntakeP", 5);
        public static final LoggedTunableNumber coralIntakeI = new LoggedTunableNumber("CoralIntake/coralIntakeI", 0);
        public static final LoggedTunableNumber coralIntakeD = new LoggedTunableNumber("CoralIntake/coralIntakeD", 0.5);

        public static final LoggedTunableNumber maxPivotV = new LoggedTunableNumber("CoralIntake/maxV", 3.5);
        public static final LoggedTunableNumber maxPivotA = new LoggedTunableNumber("CoralIntake/maxA", 10);

    }

    public static final LoggedTunableNumber pivotOffset = new LoggedTunableNumber("CoralIntake/pivotAbsoluteEncoderOffset", 0.03);
}

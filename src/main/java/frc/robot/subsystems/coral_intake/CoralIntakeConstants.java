package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import frc.lib.LoggedTunableNumber;

public class CoralIntakeConstants {
    public static final LoggedTunableNumber coralIntakeKs = new LoggedTunableNumber("coralIntakeKs", 0.03);
    public static final LoggedTunableNumber coralIntakeKg = new LoggedTunableNumber("coralIntakeKg", 0.23);
    public static final LoggedTunableNumber coralIntakeKv = new LoggedTunableNumber("coralIntakeKv", 0.8);
    public static final LoggedTunableNumber coralIntakeP = new LoggedTunableNumber("coralIntakeP", 5);
    public static final LoggedTunableNumber coralIntakeD = new LoggedTunableNumber("coralIntakeD", 0.5);

    public static final LoggedTunableNumber maxPivotV = new LoggedTunableNumber("maxV", 3.5);
    public static final LoggedTunableNumber maxPivotA = new LoggedTunableNumber("maxA", 10);

    public static final Angle pivotOffset = Radians.of(0);
}

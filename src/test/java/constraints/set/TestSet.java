package constraints.set;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.Interval;
import frc.robot.subsystems.superstructure.Constraints.RealNumberSet;

public class TestSet {
    protected boolean isValidIntervals(RealNumberSet<AngleUnit, Angle> set) {
        double previousEnd = Double.NEGATIVE_INFINITY;

        for (Interval<AngleUnit, Angle> interval : set.getIntervals()) {
            double start = interval.getStart().baseUnitMagnitude();
            double end = interval.getEnd().baseUnitMagnitude();

            if (start <= previousEnd) {
                return false;
            }

            if (end < start) {
                return false;
            }

            previousEnd = end;
        }

        return true;
    }

    protected boolean isEquivalentSets(RealNumberSet<AngleUnit, Angle> set1, RealNumberSet<AngleUnit, Angle> set2) {
        return set1.toString().equals(set2.toString());
    }
}

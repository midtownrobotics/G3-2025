package constraints;

import static frc.robot.utils.UnitUtil.max;
import static frc.robot.utils.UnitUtil.min;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.Interval;
import frc.robot.subsystems.superstructure.Constraints.RealNumberSet;

public class RealNumberTestSet extends RealNumberSet<AngleUnit, Angle> {
    public boolean containsNumber(double num) {
        return getIntervalOfValue(Units.Radians.of(num)) != null;
    }

    public RealNumberTestSet intersection(RealNumberTestSet other) {
        RealNumberSet<AngleUnit, Angle> result = super.intersection(other);
        int i = 0, j = 0;
        while (i < this.intervals.size() && j < other.intervals.size()) {
            Interval<AngleUnit, Angle> a = this.intervals.get(i);
            Interval<AngleUnit, Angle> b = other.intervals.get(j);

            // Find overlap
            Angle start = max(a.getStart(), b.getStart());
            Angle end = min(a.getEnd(), b.getEnd());

            if (start.lte(end)) { // Overlapping
                result.add(new Interval<>(start, end));
            }

            // Move to the next interval
            if (a.getEnd().lt(b.getEnd())) {
                i++;
            } else {
                j++;
            }
        }
        RealNumberTestSet output = new RealNumberTestSet();
        output.intervals = result.getIntervals();
        return output;
    }

    public RealNumberTestSet difference(RealNumberTestSet other) {
        RealNumberTestSet result = new RealNumberTestSet();
        for (Interval<AngleUnit, Angle> a : this.intervals) {
            Angle currentStart = a.getStart();
            Angle currentEnd = a.getEnd();
            for (Interval<AngleUnit, Angle> b : other.intervals) {
                if (b.getEnd().lte(currentStart)) continue;
                if (b.getStart().gte(currentEnd)) break;
                if (b.getStart().gt(currentStart)) {
                    result.add(new Interval<>(currentStart, min(b.getStart(), currentEnd)));
                }
                currentStart = max(currentStart, b.getEnd());
                if (currentStart.gte(currentEnd)) break;
            }
            if (currentStart.lt(currentEnd)) {
                result.add(new Interval<>(currentStart, currentEnd));
            }
        }
        return result;
    }
}

package frc.robot.subsystems.superstructure.Constraints;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class CircularConstraint {

    private RealNumberSet<AngleUnit, Angle> intervals;

    /**
     * Constructor for a new Circular Constraint
     */
    public CircularConstraint() {
        intervals = new RealNumberSet<>();
        intervals.add(Degrees.of(0), Degrees.of(360));
    }

    /**
     * Adds a constraint that the final angle must be between two angles
     */
    public CircularConstraint addStayInConstraint(Angle start, Angle end) {
        start = normalize(start);
        end = normalize(end);

        RealNumberSet<AngleUnit, Angle> intersectedSet = new RealNumberSet<>();
        if (end.gte(start)) {
            intersectedSet.add(start, end);
        } else {
            intersectedSet.add(Degrees.of(0), start);
            intersectedSet.add(end, Degrees.of(360));
        }

        intervals = intervals.intersection(intersectedSet);

        return this;
    }

    /**
     * Adds a constraint that the final angle must be outside of two angles
     */
    public CircularConstraint addKeepOutConstraint(Angle start, Angle end) {
        start = normalize(start);
        end = normalize(end);

        RealNumberSet<AngleUnit, Angle> intersectedSet = new RealNumberSet<>();
        if (end.gte(start)) {
            intersectedSet.add(Degrees.of(0), start);
            intersectedSet.add(end, Degrees.of(360));
        } else {
            intersectedSet.add(start, end);
        }

        intervals = intervals.intersection(intersectedSet);

        return this;
    }

    /**
     * Intersects two constraints together
     */
    public CircularConstraint addConstraint(CircularConstraint constraint) {
        intervals = intervals.intersection(constraint.intervals);
        return this;
    }

    private Angle normalize(Angle angle) {
        double degrees = angle.in(Units.Degrees);
        degrees = degrees % 360;
        if (degrees < 0) degrees += 360;
        return Units.Degrees.of(degrees);
    }

    /**
     * Checks if an angle is valid to reach under the current constraints.
     * TODO: Finish wraparound logic
     */
    public boolean isValid(Angle angle) {
        angle = normalize(angle);
        return intervals.getIntervalOfValue(angle) != null;
    }

    /**
     * Gets the angle closest to the desired angle based on the current angle and these constraints.
     * TODO: Finish method
     */
    public Angle getClosestToDesired(Angle current, Angle desired) {
        Interval<AngleUnit, Angle> interval = intervals.getIntervalOfValue(current);

        if (desired.lte(interval.end) && desired.gte(interval.start)) {
            return desired;
        }

        return desired;
    }

    // public Angle getClosestToDesiredInDirection(Angle current, Angle desired, boolean positive) {
    //     Interval<AngleUnit, Angle> interval = intervals.getIntervalOfValue(current);

    //     if (desired.lte(interval.end) && desired.gte(interval.start)) {
    //         return desired;
    //     }

    //     if (positive) {
    //         if (interval.end.lt(Degrees.of(360))) {
    //             return interval.end;
    //         }

    //         Interval<AngleUnit, Angle> wraparoundInterval = intervals.getIntervalOfValue(Degrees.of(0));
    //         if (wrap)
    //     }

    //     return desired;
    // }

    // public Angle getWraparoundDifference(Angle value1, Angle value2) {
    //     double val1 = value1.in(Degrees);
    //     double val2 = value2.in(Degrees);

    //     double difference = Math.abs(val1 - val2);

    //     if (difference > 180) return 360 - difference;
    // }
}

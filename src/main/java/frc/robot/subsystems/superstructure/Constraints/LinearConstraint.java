package frc.robot.subsystems.superstructure.Constraints;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class LinearConstraint<U extends Unit, M extends Measure<U>> {

    private RealNumberSet<U, M> intervals;

    /**
     * Constructor for a new Linear Constraint
     */
    public LinearConstraint(M start, M end) {
        intervals = new RealNumberSet<>();
        intervals.add(new Interval<>(start, end));
    }

    /**
     * Adds a constraint that the final measure must be between the two given measures
     */
    public LinearConstraint<U, M> addStayInConstraint(M start, M end) {
        RealNumberSet<U, M> intersectedSet = new RealNumberSet<>();
        intersectedSet.add(new Interval<>(start, end));

        intervals = intervals.intersection(intersectedSet);

        return this;
    }

    /**
     * Adds a constraint that the final measure must be outside of the two given measures
     */
    public LinearConstraint<U, M> addKeepOutConstraint(M start, M end) {
        RealNumberSet<U, M> intersectedSet = new RealNumberSet<>();
        intersectedSet.add(new Interval<>(start, end));

        intervals = intervals.difference(intersectedSet);

        return this;
    }

    /**
     * Intersects two constraints together
     */
    public LinearConstraint<U, M> addConstraint(LinearConstraint<U, M> constraint) {
        intervals = intervals.intersection(constraint.intervals);
        return this;
    }

    /**
     * Checks if a measure is valid to reach under the current constraints.
     */
    public boolean isValid(M value) {
        return intervals.getIntervalOfValue(value) != null;
    }

    /**
     * Gets the measure closest to the desired setpoint based on the current reading and these constraints.
     */
    public M getClosestToDesired(M current, M desired) {
        Interval<U, M> interval = intervals.getIntervalOfValue(current);

        if (interval == null) {
            return current;
        }

        if (desired.gt(current) && desired.gt(interval.getEnd())) {
            return interval.getEnd();
        }

        if (desired.lt(current) && desired.lt(interval.getStart())) {
            return interval.getStart();
        }

        return desired;
    }


}

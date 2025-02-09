package frc.robot.subsystems.superstructure.Constraints;

import static frc.robot.utils.UnitUtil.max;
import static frc.robot.utils.UnitUtil.min;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class LinearConstraint<U extends Unit, M extends Measure<U>> {

    private RealNumberSet<U, M> intervals;
    private final M lower;
    private final M upper;

    /**
     * Constructor for a new Linear Constraint
     */
    public LinearConstraint(M lower, M upper) {
        intervals = new RealNumberSet<>();
        this.lower = lower;
        this.upper = upper;
        intervals.add(new Interval<>(lower, upper));
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

        /** Case where current value is illegal. Priority is finding closest illegal value */
        if (interval == null) {
            /** Set complement gives intervals of illegal values rather than legal ones */
            RealNumberSet<U, M> inverted = intervals.complement(lower, upper);
            Interval<U, M> invertedInterval = inverted.getIntervalOfValue(current);

            /** These are two possible legal options, the start or the end. */
            M previousLegal = invertedInterval.getStart();
            M nextLegal = invertedInterval.getEnd();

            /** For cases where the lower or upper bounds are the ones considered to be legal */
            if (previousLegal.isEquivalent(lower)) return nextLegal;
            if (nextLegal.isEquivalent(upper)) return previousLegal;

            // Note: There is one case not account for here - that the entire range is illegal. Thus, this function would return the closest bound

            // If closer to previous legal, return that. Else return next legal.
            if (previousLegal.minus(current).abs(current.baseUnit()) < nextLegal.minus(current).abs(current.baseUnit())) {
                return previousLegal;
            }

            return nextLegal;
        }

        if (desired.gte(current)) {
            return min(desired, interval.getEnd());
        }

        return max(desired, interval.getStart());
    }

    @Override
    public String toString() {
        return intervals.toString();
    }
}

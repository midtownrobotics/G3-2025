package frc.robot.subsystems.superstructure.Constraints;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.utils.UnitUtil.max;
import static frc.robot.utils.UnitUtil.min;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class CircularConstraint {

    public RealNumberSet<AngleUnit, Angle> intervals;

    /**
     * Constructor for a new Circular Constraint
     */
    public CircularConstraint() {
        intervals = new RealNumberSet<>();
        intervals.add(new Interval<>(Degrees.of(0), Degrees.of(360)));
    }

    /**
     * Adds a constraint that the final angle must be between two angles
     */
    public CircularConstraint addStayInConstraint(Angle start, Angle end) {
        start = normalize(start);
        end = normalize(end);

        RealNumberSet<AngleUnit, Angle> intersectedSet = new RealNumberSet<>();
        if (end.gte(start)) {
            intersectedSet.add(new Interval<>(start, end));
        } else {
            intersectedSet.add(new Interval<>(Degrees.of(0), start));
            intersectedSet.add(new Interval<>(end, Degrees.of(360)));
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

        RealNumberSet<AngleUnit, Angle> differenceSet = new RealNumberSet<>();
        if (end.gte(start)) {
            differenceSet.add(new Interval<>(start, end));
        } else {
            differenceSet.add(new Interval<>(Degrees.of(0), start));
            differenceSet.add(new Interval<>(end, Degrees.of(360)));
        }

        intervals = intervals.difference(differenceSet);

        return this;
    }

    /**
     * Intersects two constraints together
     */
    public CircularConstraint addConstraint(CircularConstraint constraint) {
        intervals = intervals.intersection(constraint.intervals);
        return this;
    }

    /**
     * Normalizes an angle to be between 0 and 360 degrees
     */
    private Angle normalize(Angle angle) {
        double degrees = angle.in(Units.Degrees);
        degrees = degrees % 360;
        if (degrees < 0) degrees += 360;
        return Units.Degrees.of(degrees);
    }

    /**
     * Checks if an angle is valid to reach under the current constraints.
     */
    public boolean isValid(Angle angle) {
        angle = normalize(angle);
        Interval<AngleUnit, Angle> interval = getWraparoundInterval(angle);

        if (interval == null) return false;

        return interval.contains(angle) || interval.contains(angle.plus(Degrees.of(360))) || interval.contains(angle.minus(Degrees.of(360)));
    }

    /**
     * Gets the angle closest to the desired angle based on the current angle and these constraints.
     * @param current A non-normalized angle of the current position of the system
     * @param desired A normalized angle (can be not normalized) of the desired position of the system
     * @return Best angle to target based on constraints. Null if the current position is impossible.
     */
    public Angle getClosestToDesired(Angle current, Angle desired) {
        Angle delta = getDeltaToDesired(normalize(current), normalize(desired));

        if (delta == null) return null;

        return current.plus(delta);
    }

    /**
     * Gets the delta to be as close to the desired angle as possible
     * @param current A normalized angle of the current position of the system
     * @param desired A normalized angle of the desired position of the system
     * @return Best delta to target based on constraints. Null if the current position is impossible.
     */
    private Angle getDeltaToDesired(Angle current, Angle desired) {
        // Get the wraparound Interval of the current positive
        Interval<AngleUnit, Angle> interval = getWraparoundInterval(current);

        if (interval == null) return null;

        // Find the goal in the positive direction
        Angle positiveGoal = desired;
        if (desired.lt(current)) {
            positiveGoal = desired.plus(Degrees.of(360));
        }

        // Find how close we can get to that goal
        Angle positiveOutput = min(positiveGoal, interval.getEnd());

        // Find the goal in the negative direction
        Angle negativeGoal = desired;
        if (desired.gt(current)) {
            negativeGoal = desired.minus(Degrees.of(360));
        }

        // Find how close we can get to that goal
        Angle negativeOutput = max(negativeGoal, interval.getStart());

        // Calculate the delta travelled in each direction
        Angle positiveDelta = positiveOutput.minus(current);
        Angle negativeDelta = negativeOutput.minus(current);

        // Choose positive if its closer to goal
        if (negativeOutput.minus(negativeGoal).abs(Degrees) > positiveOutput.minus(positiveGoal).abs(Degrees)) {
            return positiveDelta;
        }

        // Choose negative if its closer to goal
        if (negativeOutput.minus(negativeGoal).abs(Degrees) < positiveOutput.minus(positiveGoal).abs(Degrees)) {
            return negativeDelta;
        }

        // If both are equidistance, chose whichever is closer to current position
        if (positiveDelta.abs(Degrees) < negativeDelta.abs(Degrees)) {
            return positiveDelta;
        }

        return negativeDelta;
    }

    /**
     * Returns a wraparound interval (from -360 to 720) of the given angle based on constraints.
     * For example, if current constraint is [[0, 50], [300, 360]]:
     * getWraparoundInterval(40) --> [-60, 50]
     * getWraparoundInterval(320) --> [300, 410]
     */
    private Interval<AngleUnit, Angle> getWraparoundInterval(Angle angle) {
        Interval<AngleUnit, Angle> interval = intervals.getIntervalOfValue(angle);

        if (interval == null) return null;

        Angle start = interval.getStart();
        Angle end = interval.getEnd();

        if (interval.contains(Degrees.of(360))) {
            Interval<AngleUnit, Angle> wrapAround = intervals.getIntervalOfValue(Degrees.of(0));
            if (wrapAround != null) {
                end = wrapAround.getEnd().plus(Degrees.of(360));
            }
        }

        if (interval.contains(Degrees.of(0))) {
            Interval<AngleUnit, Angle> wrapAround = intervals.getIntervalOfValue(Degrees.of(360));
            if (wrapAround != null) {
                start = wrapAround.getStart().minus(Degrees.of(360));
            }
        }

        return new Interval<AngleUnit,Angle>(start, end);
    }

    @Override
    public String toString() {
        return intervals.toString();
    }
}

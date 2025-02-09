package constraints.circular_constraint;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import constraints.set.TestSet;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import frc.robot.subsystems.superstructure.Constraints.Interval;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestAddStayInConstraint extends TestSet {
    private CircularConstraint constraint;

    @BeforeEach
    public void setUp() {
        // By default, the circular constraint is full circle [0,360]
        constraint = new CircularConstraint();
    }

    @Test
    public void testStayInConstraintNonWrapping() {
        // Using a non-wrapping interval: [90, 180]
        constraint.addStayInConstraint(Degrees.of(90), Degrees.of(180));

        // Expect a single interval: [90, 180]
        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(1, constraint.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval = constraint.intervals.getIntervals().get(0);
        assertEquals(Degrees.of(90), interval.getStart());
        assertEquals(Degrees.of(180), interval.getEnd());
    }

    @Test
    public void testStayInConstraintWrapping() {
        // Wrapping interval: start = 300, end = 60 (since 60 < 300, we expect two intervals)
        // This should produce two intervals: [0, 60] and [300, 360]
        constraint.addStayInConstraint(Degrees.of(300), Degrees.of(60));

        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(2, constraint.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval1 = constraint.intervals.getIntervals().get(0);
        Interval<AngleUnit, Angle> interval2 = constraint.intervals.getIntervals().get(1);

        assertEquals(Degrees.of(0), interval1.getStart());
        assertEquals(Degrees.of(60), interval1.getEnd());
        assertEquals(Degrees.of(300), interval2.getStart());
        assertEquals(Degrees.of(360), interval2.getEnd());
    }

    @Test
    public void testStayInConstraintFullCircle() {
        // Applying the full circle constraint [0,360] should leave the constraint unchanged
        constraint.addStayInConstraint(Degrees.of(0), Degrees.of(360));

        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(1, constraint.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval = constraint.intervals.getIntervals().get(0);
        assertEquals(Degrees.of(0), interval.getStart());
        assertEquals(Degrees.of(360), interval.getEnd());
    }

    @Test
    public void testStayInConstraintNegativeAngles() {
        // Negative angle: -90 normalizes to 270. So addStayInConstraint(-90, 90)
        // becomes equivalent to addStayInConstraint(270, 90) which should produce two intervals:
        // one from [0, 90] and one from [270, 360]
        constraint.addStayInConstraint(Degrees.of(-90), Degrees.of(90));

        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(2, constraint.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval1 = constraint.intervals.getIntervals().get(0);
        Interval<AngleUnit, Angle> interval2 = constraint.intervals.getIntervals().get(1);

        // Depending on ordering, one interval should be [0,90] and the other [270,360]
        assertEquals(Degrees.of(0), interval1.getStart());
        assertEquals(Degrees.of(90), interval1.getEnd());
        assertEquals(Degrees.of(270), interval2.getStart());
        assertEquals(Degrees.of(360), interval2.getEnd());
    }

    @Test
    public void testMultipleStayInConstraints() {
        // First, restrict to [90, 270]
        constraint.addStayInConstraint(Degrees.of(90), Degrees.of(270));
        // Then further restrict to [100, 260]
        constraint.addStayInConstraint(Degrees.of(100), Degrees.of(260));

        // Expected result is a single interval: [100, 260]
        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(1, constraint.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval = constraint.intervals.getIntervals().get(0);
        assertEquals(Degrees.of(100), interval.getStart());
        assertEquals(Degrees.of(260), interval.getEnd());
    }

    @Test
    public void testStayInConstraintNoOverlap() {
        // Create two constraints that do not overlap:
        // First, restrict to [100, 200]
        constraint.addStayInConstraint(Degrees.of(100), Degrees.of(200));
        // Then, restrict to [300, 400] (400 normalizes to 40)
        // Since [300,360] U [0,40] does not overlap with [100,200], the intersection is empty.
        constraint.addStayInConstraint(Degrees.of(300), Degrees.of(400));

        // Expect no valid intervals after intersection
        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(0, constraint.intervals.getIntervals().size());
    }
}

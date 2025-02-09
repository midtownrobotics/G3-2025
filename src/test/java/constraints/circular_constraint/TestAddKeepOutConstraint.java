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

public class TestAddKeepOutConstraint extends TestSet {
    private CircularConstraint constraint;

    @BeforeEach
    public void setUp() {
        // The circular constraint is initialized to [0,360]
        constraint = new CircularConstraint();
    }

    @Test
    public void testKeepOutConstraintNonWrapping() {
        // Remove a non-wrapping region: [90,180]
        // Expected valid intervals: [0,90] and [180,360]
        constraint.addKeepOutConstraint(Degrees.of(90), Degrees.of(180));

        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(2, constraint.intervals.getIntervals().size());

        // Since intervals are always sorted from least to greatest:
        Interval<AngleUnit, Angle> interval1 = constraint.intervals.getIntervals().get(0);
        Interval<AngleUnit, Angle> interval2 = constraint.intervals.getIntervals().get(1);
        assertEquals(Degrees.of(0), interval1.getStart());
        assertEquals(Degrees.of(90), interval1.getEnd());
        assertEquals(Degrees.of(180), interval2.getStart());
        assertEquals(Degrees.of(360), interval2.getEnd());
    }

    @Test
    public void testKeepOutConstraintTouchingLowerBoundary() {
        // Remove [0,90] should remove the beginning of the circle.
        // Expected valid interval: [90,360]
        constraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(90));

        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(1, constraint.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval = constraint.intervals.getIntervals().get(0);
        assertEquals(Degrees.of(90), interval.getStart());
        assertEquals(Degrees.of(360), interval.getEnd());
    }

    @Test
    public void testKeepOutConstraintTouchingUpperBoundary() {
        // Remove [270,360] should remove the end of the circle.
        // Expected valid interval: [0,270]
        constraint.addKeepOutConstraint(Degrees.of(270), Degrees.of(360));

        System.out.println(constraint);

        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(1, constraint.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval = constraint.intervals.getIntervals().get(0);
        assertEquals(Degrees.of(0), interval.getStart());
        assertEquals(Degrees.of(270), interval.getEnd());
    }

    @Test
    public void testKeepOutConstraintFullRemoval() {
        // Removing the full circle should leave no valid intervals.
        constraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(360));

        assertTrue(isValidIntervals(constraint.intervals));
        assertEquals(1, constraint.intervals.getIntervals().size());
    }

    @Test
    public void testKeepOutConstraintWrapping() {
        // Apply a wrapping keep-out constraint: remove angles from 300° to 60°.
        // After normalization, the method adds two intervals to remove:
        // [0,60] and [300,360]. Therefore, the valid interval should be [60,300].
        constraint.addKeepOutConstraint(Degrees.of(300), Degrees.of(60));

        assertTrue(isValidIntervals(constraint.intervals));
        // We expect a single valid interval: [60,300]
        assertEquals(1, constraint.intervals.getIntervals().size());
        Interval<AngleUnit, Angle> interval = constraint.intervals.getIntervals().get(0);
        assertEquals(Degrees.of(60), interval.getStart());
        assertEquals(Degrees.of(300), interval.getEnd());
    }
}

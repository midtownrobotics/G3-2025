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

public class TestAddConstraint extends TestSet {
    private CircularConstraint constraint1;
    private CircularConstraint constraint2;

    @BeforeEach
    public void setUp() {
        // Initialize both constraints as full circles ([0,360])
        constraint1 = new CircularConstraint();
        constraint2 = new CircularConstraint();
    }

    @Test
    public void testIntersectOverlappingConstraintsNonWrapping() {
        // Constraint1: valid range [90,270]
        constraint1.addStayInConstraint(Degrees.of(90), Degrees.of(270));
        // Constraint2: valid range [180,360]
        constraint2.addStayInConstraint(Degrees.of(180), Degrees.of(360));

        // Intersection of [90,270] and [180,360] is [180,270]
        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.intervals));
        assertEquals(1, constraint1.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval = constraint1.intervals.getIntervals().get(0);
        assertEquals(Degrees.of(180), interval.getStart());
        assertEquals(Degrees.of(270), interval.getEnd());
    }

    @Test
    public void testIntersectOverlappingConstraintsWrapping() {
        // Constraint1: valid range from addStayInConstraint(300,60) produces two intervals: [0,60] and [300,360]
        constraint1.addStayInConstraint(Degrees.of(300), Degrees.of(60));
        // Constraint2: valid range from addStayInConstraint(350,100) produces two intervals: [0,100] and [350,360]
        constraint2.addStayInConstraint(Degrees.of(350), Degrees.of(100));

        // The intersection is:
        //   Intersection of [0,60] and [0,100] → [0,60]
        //   Intersection of [300,360] and [350,360] → [350,360]
        // Expected valid intervals: [0,60] and [350,360]
        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.intervals));
        assertEquals(2, constraint1.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> first = constraint1.intervals.getIntervals().get(0);
        Interval<AngleUnit, Angle> second = constraint1.intervals.getIntervals().get(1);

        // Since intervals are sorted from least to greatest:
        assertEquals(Degrees.of(0), first.getStart());
        assertEquals(Degrees.of(60), first.getEnd());
        assertEquals(Degrees.of(350), second.getStart());
        assertEquals(Degrees.of(360), second.getEnd());
    }

    @Test
    public void testIntersectNonOverlappingConstraints() {
        // Constraint1: valid range [90,150]
        constraint1.addStayInConstraint(Degrees.of(90), Degrees.of(150));
        // Constraint2: valid range [210,270]
        constraint2.addStayInConstraint(Degrees.of(210), Degrees.of(270));

        // Intersection of [90,150] and [210,270] is empty
        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.intervals));
        assertEquals(0, constraint1.intervals.getIntervals().size());
    }

    @Test
    public void testIntersectWithFullCircle() {
        // Constraint1: restrict to [120,240]
        constraint1.addStayInConstraint(Degrees.of(120), Degrees.of(240));
        // Constraint2 remains full circle ([0,360])
        // Their intersection should be [120,240]
        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.intervals));
        assertEquals(1, constraint1.intervals.getIntervals().size());

        Interval<AngleUnit, Angle> interval = constraint1.intervals.getIntervals().get(0);
        assertEquals(Degrees.of(120), interval.getStart());
        assertEquals(Degrees.of(240), interval.getEnd());
    }
}

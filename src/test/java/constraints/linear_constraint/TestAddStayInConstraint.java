package constraints.linear_constraint;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import constraints.set.TestSet;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestAddStayInConstraint extends TestSet {
    private LinearConstraint<AngleUnit, Angle> constraint;

    @BeforeEach
    void setUp() {
        constraint = new LinearConstraint<>(Degrees.of(0), Degrees.of(100));
    }

    @Test
    public void testStayInConstraintWithinBounds() {
        constraint.addStayInConstraint(Degrees.of(20), Degrees.of(80));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(1, constraint.getSet().getIntervals().size());
        assertEquals(Degrees.of(20), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(80), constraint.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testStayInConstraintReducingIntervalTwice() {
        constraint.addStayInConstraint(Degrees.of(20), Degrees.of(80));
        constraint.addStayInConstraint(Degrees.of(30), Degrees.of(70));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(1, constraint.getSet().getIntervals().size());
        assertEquals(Degrees.of(30), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(70), constraint.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testStayInConstraintNoOverlap() {
        constraint.addStayInConstraint(Degrees.of(120), Degrees.of(150));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(0, constraint.getSet().getIntervals().size()); // Entire range should be removed
    }

    @Test
    public void testStayInConstraintMatchingExistingBounds() {
        constraint.addStayInConstraint(Degrees.of(0), Degrees.of(100));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(1, constraint.getSet().getIntervals().size());
        assertEquals(Degrees.of(0), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(100), constraint.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testStayInConstraintTouchingEdges() {
        constraint.addStayInConstraint(Degrees.of(0), Degrees.of(50));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(1, constraint.getSet().getIntervals().size());
        assertEquals(Degrees.of(0), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(50), constraint.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testStayInConstraintInsideExistingRange() {
        constraint.addStayInConstraint(Degrees.of(10), Degrees.of(90));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(1, constraint.getSet().getIntervals().size());
        assertEquals(Degrees.of(10), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(90), constraint.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testStayInConstraintMultipleDisjointIntervals() {
        constraint.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint.addStayInConstraint(Degrees.of(30), Degrees.of(40));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(1, constraint.getSet().getIntervals().size());
        assertEquals(Degrees.of(30), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(40), constraint.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testStayInConstraintInvalidRange() {
        assertThrows(IllegalArgumentException.class, () -> constraint.addStayInConstraint(Degrees.of(50), Degrees.of(30)));
    }
}

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

public class TestAddKeepOutConstraint extends TestSet {
    private LinearConstraint<AngleUnit, Angle> constraint;

    @BeforeEach
    void setUp() {
        constraint = new LinearConstraint<>(Degrees.of(0), Degrees.of(100));
    }

    @Test
    public void testKeepOutConstraintWithinBounds() {
        constraint.addKeepOutConstraint(Degrees.of(20), Degrees.of(80));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(2, constraint.getSet().getIntervals().size());

        assertEquals(Degrees.of(0), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(20), constraint.getSet().getIntervals().get(0).getEnd());

        assertEquals(Degrees.of(80), constraint.getSet().getIntervals().get(1).getStart());
        assertEquals(Degrees.of(100), constraint.getSet().getIntervals().get(1).getEnd());
    }

    @Test
    public void testKeepOutConstraintCoveringEntireRange() {
        constraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(100));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(0, constraint.getSet().getIntervals().size()); // Entire range should be removed
    }

    @Test
    public void testKeepOutConstraintTouchingEdges() {
        constraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(50));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(1, constraint.getSet().getIntervals().size());

        assertEquals(Degrees.of(50), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(100), constraint.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testKeepOutConstraintInsideExistingRange() {
        constraint.addKeepOutConstraint(Degrees.of(10), Degrees.of(90));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(2, constraint.getSet().getIntervals().size());

        assertEquals(Degrees.of(0), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(10), constraint.getSet().getIntervals().get(0).getEnd());

        assertEquals(Degrees.of(90), constraint.getSet().getIntervals().get(1).getStart());
        assertEquals(Degrees.of(100), constraint.getSet().getIntervals().get(1).getEnd());
    }

    @Test
    public void testKeepOutConstraintMultipleDisjointIntervals() {
        constraint.addKeepOutConstraint(Degrees.of(10), Degrees.of(50));
        constraint.addKeepOutConstraint(Degrees.of(60), Degrees.of(90));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(3, constraint.getSet().getIntervals().size());

        assertEquals(Degrees.of(0), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(10), constraint.getSet().getIntervals().get(0).getEnd());

        assertEquals(Degrees.of(50), constraint.getSet().getIntervals().get(1).getStart());
        assertEquals(Degrees.of(60), constraint.getSet().getIntervals().get(1).getEnd());

        assertEquals(Degrees.of(90), constraint.getSet().getIntervals().get(2).getStart());
        assertEquals(Degrees.of(100), constraint.getSet().getIntervals().get(2).getEnd());
    }

    @Test
    public void testKeepOutConstraintMatchingExistingBounds() {
        constraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(100));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(0, constraint.getSet().getIntervals().size()); // Entire range should be removed
    }

    @Test
    public void testKeepOutConstraintNoEffect() {
        constraint.addKeepOutConstraint(Degrees.of(110), Degrees.of(120));

        assertTrue(isValidIntervals(constraint.getSet()));
        assertEquals(1, constraint.getSet().getIntervals().size());
        assertEquals(Degrees.of(0), constraint.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(100), constraint.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testKeepOutConstraintInvalidRange() {
        assertThrows(IllegalArgumentException.class, () -> constraint.addKeepOutConstraint(Degrees.of(50), Degrees.of(30)));
    }
}

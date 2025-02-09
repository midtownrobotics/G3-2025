package constraints.linear_constraint;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import constraints.set.TestSet;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestAddConstraint extends TestSet {
    private LinearConstraint<AngleUnit, Angle> constraint1;
    private LinearConstraint<AngleUnit, Angle> constraint2;

    @BeforeEach
    void setUp() {
        constraint1 = new LinearConstraint<>(Degrees.of(0), Degrees.of(100));
        constraint2 = new LinearConstraint<>(Degrees.of(0), Degrees.of(100));
    }

    @Test
    public void testAddConstraintWithinBounds() {
        constraint1.addStayInConstraint(Degrees.of(20), Degrees.of(80));
        constraint2.addStayInConstraint(Degrees.of(40), Degrees.of(60));

        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.getSet()));
        assertEquals(1, constraint1.getSet().getIntervals().size());
        assertEquals(Degrees.of(40), constraint1.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(60), constraint1.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testAddConstraintWithPartialOverlap() {
        constraint1.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint2.addStayInConstraint(Degrees.of(30), Degrees.of(70));

        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.getSet()));
        assertEquals(1, constraint1.getSet().getIntervals().size());
        assertEquals(Degrees.of(30), constraint1.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(50), constraint1.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testAddConstraintWithDisjointIntervals() {
        constraint1.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint1.addKeepOutConstraint(Degrees.of(20), Degrees.of(40));

        constraint2.addStayInConstraint(Degrees.of(15), Degrees.of(45));

        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.getSet()));
        assertEquals(2, constraint1.getSet().getIntervals().size());

        assertEquals(Degrees.of(15), constraint1.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(20), constraint1.getSet().getIntervals().get(0).getEnd());

        assertEquals(Degrees.of(40), constraint1.getSet().getIntervals().get(1).getStart());
        assertEquals(Degrees.of(45), constraint1.getSet().getIntervals().get(1).getEnd());
    }

    @Test
    public void testAddConstraintThatRemovesAllIntervals() {
        constraint1.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint2.addStayInConstraint(Degrees.of(60), Degrees.of(80));

        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.getSet()));
        assertEquals(0, constraint1.getSet().getIntervals().size()); // No overlap means empty set
    }

    @Test
    public void testAddConstraintWithOneEmptySet() {
        constraint1.addStayInConstraint(Degrees.of(10), Degrees.of(50));

        constraint2 = new LinearConstraint<>(Degrees.of(0), Degrees.of(100)); // Empty constraint

        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.getSet()));
        assertEquals(1, constraint1.getSet().getIntervals().size());
        assertEquals(Degrees.of(10), constraint1.getSet().getIntervals().get(0).getStart());
        assertEquals(Degrees.of(50), constraint1.getSet().getIntervals().get(0).getEnd());
    }

    @Test
    public void testAddConstraintWithBothEmptySets() {
        constraint1 = new LinearConstraint<>(Degrees.of(0), Degrees.of(100));
        constraint2 = new LinearConstraint<>(Degrees.of(0), Degrees.of(100));

        // Manually empty the sets
        constraint1.addKeepOutConstraint(Degrees.of(0), Degrees.of(100));
        constraint2.addKeepOutConstraint(Degrees.of(0), Degrees.of(100));

        constraint1.addConstraint(constraint2);

        assertTrue(isValidIntervals(constraint1.getSet()));
        assertEquals(0, constraint1.getSet().getIntervals().size()); // Now truly empty
    }

}

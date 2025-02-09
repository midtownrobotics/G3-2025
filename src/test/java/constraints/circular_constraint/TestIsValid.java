package constraints.circular_constraint;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import constraints.set.TestSet;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestIsValid extends TestSet {
    private CircularConstraint constraint;

    @BeforeEach
    public void setUp() {
        // By default, CircularConstraint covers [0,360]
        constraint = new CircularConstraint();
    }

    @Test
    public void testIsValidFullCircle() {
        // With no additional constraints, any angle (after normalization) should be valid.
        assertTrue(constraint.isValid(Degrees.of(45)));
        assertTrue(constraint.isValid(Degrees.of(-30)));  // Normalizes to 330
        assertTrue(constraint.isValid(Degrees.of(400)));  // Normalizes to 40
    }

    @Test
    public void testIsValidWithStayInConstraint() {
        // Restrict valid region to [90,270]
        constraint.addStayInConstraint(Degrees.of(90), Degrees.of(270));

        // Angles within [90,270] are valid.
        assertTrue(constraint.isValid(Degrees.of(100)));
        assertTrue(constraint.isValid(Degrees.of(90)));   // Boundary
        assertTrue(constraint.isValid(Degrees.of(270)));  // Boundary

        // Angles outside [90,270] are invalid.
        assertFalse(constraint.isValid(Degrees.of(80)));
        assertFalse(constraint.isValid(Degrees.of(300)));
    }

    @Test
    public void testIsValidWithKeepOutConstraint() {
        // Remove the range [90,270] from the full circle.
        constraint.addKeepOutConstraint(Degrees.of(90), Degrees.of(270));

        // Valid angles are in the union of [0,90] and [270,360].
        assertTrue(constraint.isValid(Degrees.of(45)));
        // Assuming boundaries are included:
        assertTrue(constraint.isValid(Degrees.of(90)));
        assertTrue(constraint.isValid(Degrees.of(270)));
        assertTrue(constraint.isValid(Degrees.of(315)));

        // An angle strictly inside the removed zone should be invalid.
        assertFalse(constraint.isValid(Degrees.of(180)));
    }

    @Test
    public void testIsValidWithComplexConstraints() {
        // First, restrict the valid region to [0,180] via a stay-in constraint,
        // then remove a subrange [90,100] via a keep-out constraint.
        // Expected valid regions: [0,90] and [100,180]
        constraint.addStayInConstraint(Degrees.of(0), Degrees.of(180));
        constraint.addKeepOutConstraint(Degrees.of(90), Degrees.of(100));

        // Valid angles:
        assertTrue(constraint.isValid(Degrees.of(50)));
        // Depending on your boundary rules, if boundaries of the keep-out are closed:
        assertTrue(constraint.isValid(Degrees.of(90)));
        assertTrue(constraint.isValid(Degrees.of(100)));
        assertTrue(constraint.isValid(Degrees.of(150)));

        // An angle strictly inside the removed region should be invalid.
        assertFalse(constraint.isValid(Degrees.of(95)));
    }

    @Test
    public void testIsValidAfterMultipleConstraints() {
        // Remove a wrapping range from the full circle. For example,
        // remove [350,10] so that valid angles are [10,350].
        constraint.addKeepOutConstraint(Degrees.of(350), Degrees.of(10));

        // Valid angles:
        assertTrue(constraint.isValid(Degrees.of(20)));
        assertTrue(constraint.isValid(Degrees.of(345)));

        // Invalid angles (normalized within the removed region):
        assertFalse(constraint.isValid(Degrees.of(5)));    // 5 is between 350 and 10 after wraparound
        assertFalse(constraint.isValid(Degrees.of(355)));  // 355 is also within the removed zone
    }
}

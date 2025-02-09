package constraints.linear_constraint;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import constraints.set.TestSet;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestIsValid extends TestSet {
    private LinearConstraint<AngleUnit, Angle> constraint;

    @BeforeEach
    void setUp() {
        constraint = new LinearConstraint<>(Degrees.of(0), Degrees.of(100));
    }

    @Test
    public void testIsValidWithinFullRange() {
        assertTrue(constraint.isValid(Degrees.of(50)));
        assertTrue(constraint.isValid(Degrees.of(0)));
        assertTrue(constraint.isValid(Degrees.of(100)));
    }

    @Test
    public void testIsValidWithinStayInConstraint() {
        constraint.addStayInConstraint(Degrees.of(20), Degrees.of(80));

        assertTrue(constraint.isValid(Degrees.of(50)));
        assertTrue(constraint.isValid(Degrees.of(20)));
        assertTrue(constraint.isValid(Degrees.of(80)));

        assertFalse(constraint.isValid(Degrees.of(10)));
        assertFalse(constraint.isValid(Degrees.of(90)));
    }

    @Test
    public void testIsValidWithKeepOutConstraint() {
        constraint.addKeepOutConstraint(Degrees.of(30), Degrees.of(70));

        assertTrue(constraint.isValid(Degrees.of(20)));
        assertTrue(constraint.isValid(Degrees.of(30)));
        assertTrue(constraint.isValid(Degrees.of(70)));
        assertTrue(constraint.isValid(Degrees.of(80)));

        assertFalse(constraint.isValid(Degrees.of(50)));
    }


    @Test
    public void testIsValidWithDisjointIntervals() {
        constraint.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint.addKeepOutConstraint(Degrees.of(20), Degrees.of(40));

        assertTrue(constraint.isValid(Degrees.of(15)));
        assertTrue(constraint.isValid(Degrees.of(45)));

        assertFalse(constraint.isValid(Degrees.of(25)));
        assertFalse(constraint.isValid(Degrees.of(35)));
    }

    @Test
    public void testIsValidWithEmptyConstraint() {
        constraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(100)); // Removes all intervals

        assertFalse(constraint.isValid(Degrees.of(50)));
        assertFalse(constraint.isValid(Degrees.of(0)));
        assertFalse(constraint.isValid(Degrees.of(100)));
    }
}

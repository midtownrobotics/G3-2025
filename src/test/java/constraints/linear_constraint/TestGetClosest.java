package constraints.linear_constraint;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import constraints.set.TestSet;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestGetClosest extends TestSet {
    private LinearConstraint<AngleUnit, Angle> constraint;

    @BeforeEach
    void setUp() {
        constraint = new LinearConstraint<>(Degrees.of(0), Degrees.of(100));
    }

    @Test
    public void testClosestWhenCurrentAndDesiredAreValid() {
        constraint.addStayInConstraint(Degrees.of(20), Degrees.of(80));

        // ✅ Test standard cases where current and desired are within the interval
        assertEquals(Degrees.of(60), constraint.getClosestToDesired(Degrees.of(50), Degrees.of(60)));
        assertEquals(Degrees.of(70), constraint.getClosestToDesired(Degrees.of(30), Degrees.of(70)));

        // ✅ Test case where current is at lower bound and desired is within range
        assertEquals(Degrees.of(30), constraint.getClosestToDesired(Degrees.of(20), Degrees.of(30)));

        // ✅ Test case where current is at upper bound and desired is within range
        assertEquals(Degrees.of(70), constraint.getClosestToDesired(Degrees.of(80), Degrees.of(70)));

        // ✅ Test when desired exceeds the valid range → Should clamp to interval end
        assertEquals(Degrees.of(80), constraint.getClosestToDesired(Degrees.of(50), Degrees.of(90))); // Desired is out of range

        // ✅ Test when desired is lower than the valid range → Should clamp to interval start
        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(50), Degrees.of(10))); // Desired is out of range
    }


    @Test
    public void testClosestWhenCurrentIsOutOfBoundsAndDesiredIsValid() {
        constraint.addStayInConstraint(Degrees.of(20), Degrees.of(80));

        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(10), Degrees.of(50)));
        assertEquals(Degrees.of(80), constraint.getClosestToDesired(Degrees.of(90), Degrees.of(60)));
    }

    @Test
    public void testClosestWhenCurrentAndDesiredAreOutOfBounds() {
        constraint.addStayInConstraint(Degrees.of(20), Degrees.of(80));

        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(10), Degrees.of(15)));
        assertEquals(Degrees.of(80), constraint.getClosestToDesired(Degrees.of(90), Degrees.of(85)));
    }

    @Test
    public void testClosestWithKeepOutConstraint() {
        constraint.addKeepOutConstraint(Degrees.of(30), Degrees.of(70));

        // ✅ Case 1: Current is VALID, Desired is INSIDE Keep-Out
        assertEquals(Degrees.of(30), constraint.getClosestToDesired(Degrees.of(20), Degrees.of(50))); // Desired in [30,70], should clamp to 30
        assertEquals(Degrees.of(70), constraint.getClosestToDesired(Degrees.of(80), Degrees.of(50))); // Desired in [30,70], should clamp to 70

        // ✅ Case 2: Current is INSIDE Keep-Out, Desired is OUTSIDE
        assertEquals(Degrees.of(30), constraint.getClosestToDesired(Degrees.of(45), Degrees.of(20))); // Should snap to 30
        assertEquals(Degrees.of(70), constraint.getClosestToDesired(Degrees.of(55), Degrees.of(90))); // Should snap to 70

        // ✅ Case 3: Current is INSIDE Keep-Out, Desired is ALSO INSIDE Keep-Out
        assertEquals(Degrees.of(30), constraint.getClosestToDesired(Degrees.of(45), Degrees.of(40))); // Both values are invalid, should return closest boundary
    }


    @Test
    public void testClosestWithDisjointIntervals() {
        constraint.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint.addKeepOutConstraint(Degrees.of(20), Degrees.of(40));

        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(25), Degrees.of(10)));
        assertEquals(Degrees.of(40), constraint.getClosestToDesired(Degrees.of(35), Degrees.of(50)));
    }


    @Test
    public void testClosestWithFullyRemovedRange() {
        constraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(100));

        assertEquals(Degrees.of(50), constraint.getClosestToDesired(Degrees.of(50), Degrees.of(25)));
        assertEquals(Degrees.of(50), constraint.getClosestToDesired(Degrees.of(50), Degrees.of(75)));
    }

    @Test
    public void testClosestWhenCurrentIsOnBoundary() {
        constraint.addStayInConstraint(Degrees.of(20), Degrees.of(80));

        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(20), Degrees.of(10)));
        assertEquals(Degrees.of(80), constraint.getClosestToDesired(Degrees.of(80), Degrees.of(90)));
    }

    @Test
    public void testClosestWhenCurrentIsInRemovedRegion() {
        constraint.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint.addKeepOutConstraint(Degrees.of(20), Degrees.of(40));

        // ✅ Closest valid values should be 20 and 40, NOT 15
        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(25), Degrees.of(45))); // Should snap to 20
        assertEquals(Degrees.of(40), constraint.getClosestToDesired(Degrees.of(35), Degrees.of(10))); // Should snap to 40
    }

    @Test
    public void testClosestWhenCurrentIsAtKeepOutBoundary() {
        constraint.addKeepOutConstraint(Degrees.of(30), Degrees.of(70));

        // ✅ If boundaries are included, current is valid → should return desired directly
        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(30), Degrees.of(20)));
        assertEquals(Degrees.of(90), constraint.getClosestToDesired(Degrees.of(70), Degrees.of(90)));
    }

    @Test
    public void testClosestWhenDesiredIsOnBoundaryOfDisjointIntervals() {
        constraint.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint.addKeepOutConstraint(Degrees.of(20), Degrees.of(40));

        // ✅ Desired is exactly on a boundary of a valid interval → should return `desired`
        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(15), Degrees.of(20)));
        assertEquals(Degrees.of(40), constraint.getClosestToDesired(Degrees.of(45), Degrees.of(40)));
    }

    @Test
    public void testClosestWhenBothCurrentAndDesiredAreAtDifferentDisjointIntervals() {
        constraint.addStayInConstraint(Degrees.of(10), Degrees.of(50));
        constraint.addKeepOutConstraint(Degrees.of(20), Degrees.of(40));

        // ✅ Current in [10,20], desired in [40,50] → Move as close as possible, but stay in [10,20]
        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(15), Degrees.of(40)));

        // ✅ Current in [40,50], desired in [10,20] → Move as close as possible, but stay in [40,50]
        assertEquals(Degrees.of(40), constraint.getClosestToDesired(Degrees.of(45), Degrees.of(20)));

        // ✅ Current in [10,20], desired outside all valid zones → Move to closest valid boundary
        assertEquals(Degrees.of(20), constraint.getClosestToDesired(Degrees.of(15), Degrees.of(30))); // 30 is invalid → Snap to 20
        assertEquals(Degrees.of(40), constraint.getClosestToDesired(Degrees.of(45), Degrees.of(30))); // 30 is invalid → Snap to 40
    }


}

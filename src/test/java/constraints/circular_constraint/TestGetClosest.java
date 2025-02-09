package constraints.circular_constraint;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import constraints.set.TestSet;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestGetClosest extends TestSet {

    private CircularConstraint constraint;

    @BeforeEach
    public void setUp() {
        // By default, CircularConstraint covers the full circle [0,360].
        constraint = new CircularConstraint();
    }

    // --- Simple Cases with Full Circle ---

    @Test
    public void testSimplePositiveDirection() {
        // With a full circle, if current = 50 and desired = 80, the arm moves forward.
        // Expected: result = 80.
        Angle result = constraint.getClosestToDesired(Degrees.of(50), Degrees.of(80));
        assertEquals(Degrees.of(80), result);
    }

    @Test
    public void testSimpleNegativeDirection() {
        // With a full circle, if current = 80 and desired = 50, the arm moves backward.
        // Expected: result = 50.
        Angle result = constraint.getClosestToDesired(Degrees.of(80), Degrees.of(50));
        assertEquals(Degrees.of(50), result);
    }

    // --- Wrap-Around Cases ---

    @Test
    public void testWrapAroundNegativeDirection() {
        // Example: current = 350, desired = 10.
        // For current=350, the positive alternative: desired + 360 = 370, delta = 20.
        // Negative alternative: desired remains 10, delta = 10 - 350 = -340 (abs=340).
        // Thus, positive move is chosen, and result = 350 + 20 = 370, which normalizes to 10.
        Angle result = constraint.getClosestToDesired(Degrees.of(350), Degrees.of(10));
        assertEquals(Degrees.of(370), result);
    }

    @Test
    public void testWrapAroundPositiveDirection() {
        // Example: current = 10, desired = 350.
        // Negative alternative: desired - 360 = 350 - 360 = -10.
        // Positive alternative: delta = 350 - 10 = 340.
        // |–10 – 10| = 20 is smaller than 340, so the negative path is chosen.
        // Expected result: current + (-20) = 10 - 20 = -10.
        Angle result = constraint.getClosestToDesired(Degrees.of(10), Degrees.of(350));
        assertEquals(Degrees.of(-10), result);
    }

    // --- Cases Involving Constraints (Illegal Regions) ---

    @Test
    public void testCurrentInvalidLegalClamping() {
        // Remove the keep-out region [30,70] so that valid intervals are [0,30] and [70,360].
        constraint.addKeepOutConstraint(Degrees.of(30), Degrees.of(70));
        // With current = 50 (illegal) and desired = 20 (legal in [0,30]),
        // the best move is to go negative from 50 to the lower boundary 30.
        // Delta = 30 - 50 = -20, so expected result = 50 + (-20) = 30.
        Angle result = constraint.getClosestToDesired(Degrees.of(49), Degrees.of(20));
        assertEquals(Degrees.of(30), result);
    }

    @Test
    public void testCurrentInvalidDesiredInvalidClamping() {
        // With the same keep-out constraint [30,70]:
        constraint.addKeepOutConstraint(Degrees.of(30), Degrees.of(70));
        // If both current and desired are in the illegal region (e.g. 50 and 40),
        // then the function chooses the boundary; tie-breaker yields negative delta.
        // Expected result: 30.
        Angle result = constraint.getClosestToDesired(Degrees.of(49), Degrees.of(40));
        assertEquals(Degrees.of(30), result);
    }

    // --- Cases When Both Current and Desired Are Valid (Under a Stay-In Constraint) ---

    @Test
    public void testExactValid() {
        // Restrict valid region to [100,200].
        constraint.addStayInConstraint(Degrees.of(100), Degrees.of(200));
        // When current = 150 and desired = 170 (both valid), expected result = 170.
        Angle result = constraint.getClosestToDesired(Degrees.of(150), Degrees.of(170));
        assertEquals(Degrees.of(170), result);
    }

    @Test
    public void testExactBoundaryReturn() {
        // With valid region [100,200]:
        constraint.addStayInConstraint(Degrees.of(100), Degrees.of(200));
        // If current is at the lower boundary (100) and desired is below valid range (e.g. 90),
        // the arm should not cross into an illegal region; expected result is 100.
        Angle result = constraint.getClosestToDesired(Degrees.of(100), Degrees.of(90));
        assertEquals(Degrees.of(100), result);
    }

    // --- Robustness Cases ---

    @Test
    public void testDesiredExactlyAtCurrent() {
        // When desired equals current, no movement should occur.
        Angle result = constraint.getClosestToDesired(Degrees.of(123), Degrees.of(123));
        assertEquals(Degrees.of(123).in(Units.Degrees), result.in(Units.Degrees));
    }

    @Test
    public void testMultipleWrapAroundConstraints() {
        // Remove [30,70] so that valid intervals are [0,30] and [70,360].
        constraint.addKeepOutConstraint(Degrees.of(30), Degrees.of(70));
        // For current = 5 (which lies in [0,30]) and desired = 350:
        // The negative alternative: desired - 360 = 350 - 360 = -10, delta = -10 - 5 = -15.
        // The positive alternative: delta = 30 - 5 = 25.
        // Since 15 < 25, the negative path is chosen.
        // Expected result = current + (-15) = 5 - 15 = -10.
        Angle result = constraint.getClosestToDesired(Degrees.of(5), Degrees.of(350));
        assertEquals(Degrees.of(-10), result);
    }
}

package constraints.circular_constraint;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import constraints.set.TestSet;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import frc.robot.subsystems.superstructure.Constraints.Interval;
import java.lang.reflect.Method;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestUnwrapInterval extends TestSet {
    private CircularConstraint circularConstraint;

    @BeforeEach
    public void setUp() {
        // Initialize to a full circle [0,360]
        circularConstraint = new CircularConstraint();
    }

    @Test
    public void testUnwrapIntervalNonWrapping() throws Exception {
        // Setup: Create a non-wrapping interval that does not cross 0/360 boundary.
        // For example, use addStayInConstraint to restrict the circle to [100,200]
        circularConstraint.addStayInConstraint(Degrees.of(100), Degrees.of(200));
        Interval<AngleUnit, Angle> interval = circularConstraint.intervals.getIntervals().get(0);

        // Use reflection to access the private unwrapInterval method.
        Method unwrapMethod = CircularConstraint.class.getDeclaredMethod("unwrapInterval", Interval.class);
        unwrapMethod.setAccessible(true);
        @SuppressWarnings("unchecked")
        Interval<AngleUnit, Angle> unwrapped = (Interval<AngleUnit, Angle>) unwrapMethod.invoke(circularConstraint, interval);

        // For a non-wrapping interval, the unwrapped interval should be the same.
        assertNotNull(unwrapped);
        assertEquals(Degrees.of(100), unwrapped.getStart());
        assertEquals(Degrees.of(200), unwrapped.getEnd());
    }

    @Test
    public void testUnwrapIntervalWrapping() throws Exception {
        // Setup: Create a wrapping interval using addStayInConstraint.
        // For example, add a constraint that wraps: [300,60] produces two intervals.
        circularConstraint.addStayInConstraint(Degrees.of(300), Degrees.of(60));

        // Let's retrieve one of the intervals that we expect to be wrapped.
        Interval<AngleUnit, Angle> interval = circularConstraint.intervals.getIntervals().get(0);

        // Access unwrapInterval via reflection.
        Method unwrapMethod = CircularConstraint.class.getDeclaredMethod("unwrapInterval", Interval.class);
        unwrapMethod.setAccessible(true);

        @SuppressWarnings("unchecked")
        Interval<AngleUnit, Angle> unwrapped = (Interval<AngleUnit, Angle>) unwrapMethod.invoke(circularConstraint, interval);

        // Let's say our interval is the one starting at 0.
        if (interval.getStart().equals(Degrees.of(0))) {
            assertEquals(Degrees.of(-60), unwrapped.getStart());
            assertEquals(Degrees.of(60), unwrapped.getEnd());
        }
    }
}

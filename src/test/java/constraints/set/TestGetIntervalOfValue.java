package constraints.set;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.Interval;
import frc.robot.subsystems.superstructure.Constraints.RealNumberSet;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestGetIntervalOfValue extends TestSet {
    private RealNumberSet<AngleUnit, Angle> set;

    @BeforeEach
    void setUp() {
        set = new RealNumberSet<>();
    }

    @Test
    public void testValueInsideSingleInterval() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(30)));

        Interval<AngleUnit, Angle> interval = set.getIntervalOfValue(Degrees.of(20));

        assertNotNull(interval);
        assertEquals(Degrees.of(10), interval.getStart());
        assertEquals(Degrees.of(30), interval.getEnd());
    }

    @Test
    public void testValueAtIntervalStart() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(30)));

        Interval<AngleUnit, Angle> interval = set.getIntervalOfValue(Degrees.of(10));

        assertNotNull(interval);
        assertEquals(Degrees.of(10), interval.getStart());
        assertEquals(Degrees.of(30), interval.getEnd());
    }

    @Test
    public void testValueAtIntervalEnd() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(30)));

        Interval<AngleUnit, Angle> interval = set.getIntervalOfValue(Degrees.of(30));

        assertNotNull(interval);
        assertEquals(Degrees.of(10), interval.getStart());
        assertEquals(Degrees.of(30), interval.getEnd());
    }

    @Test
    public void testValueOutsideAnyInterval() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(30)));

        Interval<AngleUnit, Angle> interval = set.getIntervalOfValue(Degrees.of(40));

        assertNull(interval); // Should return null if value is outside any interval
    }

    @Test
    public void testValueBetweenDisjointIntervals() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(20)));
        set.add(new Interval<>(Degrees.of(30), Degrees.of(40)));

        Interval<AngleUnit, Angle> interval = set.getIntervalOfValue(Degrees.of(25));

        assertNull(interval); // Should return null since 25 is between two disjoint intervals
    }

    @Test
    public void testValueInsideMultipleIntervals() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(20)));
        set.add(new Interval<>(Degrees.of(30), Degrees.of(40)));

        Interval<AngleUnit, Angle> interval1 = set.getIntervalOfValue(Degrees.of(15));
        Interval<AngleUnit, Angle> interval2 = set.getIntervalOfValue(Degrees.of(35));

        assertNotNull(interval1);
        assertEquals(Degrees.of(10), interval1.getStart());
        assertEquals(Degrees.of(20), interval1.getEnd());

        assertNotNull(interval2);
        assertEquals(Degrees.of(30), interval2.getStart());
        assertEquals(Degrees.of(40), interval2.getEnd());
    }

    @Test
    public void testEmptySetReturnsNull() {
        Interval<AngleUnit, Angle> interval = set.getIntervalOfValue(Degrees.of(15));

        assertNull(interval); // Should return null if there are no intervals
    }

    @Test
    public void testValueBeforeAllIntervals() {
        set.add(new Interval<>(Degrees.of(20), Degrees.of(30)));

        Interval<AngleUnit, Angle> interval = set.getIntervalOfValue(Degrees.of(10));

        assertNull(interval); // Should return null since 10 is before any interval
    }

    @Test
    public void testValueAfterAllIntervals() {
        set.add(new Interval<>(Degrees.of(20), Degrees.of(30)));

        Interval<AngleUnit, Angle> interval = set.getIntervalOfValue(Degrees.of(40));

        assertNull(interval); // Should return null since 40 is after all intervals
    }
}

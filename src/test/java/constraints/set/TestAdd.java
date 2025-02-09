package constraints.set;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.Interval;
import frc.robot.subsystems.superstructure.Constraints.RealNumberSet;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestAdd extends TestSet {
    private RealNumberSet<AngleUnit, Angle> realNumberSet;

    @BeforeEach
    public void setUp() {
        realNumberSet = new RealNumberSet<>();
    }

    @Test
    public void testAddNonOverlappingIntervals() {
        Interval<AngleUnit, Angle> interval1 = new Interval<>(Degrees.of(10), Degrees.of(30));
        Interval<AngleUnit, Angle> interval2 = new Interval<>(Degrees.of(40), Degrees.of(60));

        realNumberSet.add(interval1);
        realNumberSet.add(interval2);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(2, intervals.size());
        assertEquals(interval1, intervals.get(0));
        assertEquals(interval2, intervals.get(1));
    }

    @Test
    public void testAddOverlappingIntervals() {
        Interval<AngleUnit, Angle> interval1 = new Interval<>(Degrees.of(10), Degrees.of(40));
        Interval<AngleUnit, Angle> interval2 = new Interval<>(Degrees.of(30), Degrees.of(50));

        realNumberSet.add(interval1);
        realNumberSet.add(interval2);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(1, intervals.size());
        assertEquals(Degrees.of(10), intervals.get(0).getStart());
        assertEquals(Degrees.of(50), intervals.get(0).getEnd());
    }

    @Test
    public void testAddIntervalWithinExisting() {
        Interval<AngleUnit, Angle> interval1 = new Interval<>(Degrees.of(10), Degrees.of(50));
        Interval<AngleUnit, Angle> interval2 = new Interval<>(Degrees.of(20), Degrees.of(40));

        realNumberSet.add(interval1);
        realNumberSet.add(interval2);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(1, intervals.size());
        assertEquals(interval1, intervals.get(0));
    }

    @Test
    public void testAddAdjacentIntervals() {
        Interval<AngleUnit, Angle> interval1 = new Interval<>(Degrees.of(10), Degrees.of(30));
        Interval<AngleUnit, Angle> interval2 = new Interval<>(Degrees.of(30), Degrees.of(50));

        realNumberSet.add(interval1);
        realNumberSet.add(interval2);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(1, intervals.size());
        assertEquals(Degrees.of(10), intervals.get(0).getStart());
        assertEquals(Degrees.of(50), intervals.get(0).getEnd());
    }

    @Test
    public void testAddMultipleMergingIntervals() {
        Interval<AngleUnit, Angle> interval1 = new Interval<>(Degrees.of(10), Degrees.of(25));
        Interval<AngleUnit, Angle> interval2 = new Interval<>(Degrees.of(20), Degrees.of(35));
        Interval<AngleUnit, Angle> interval3 = new Interval<>(Degrees.of(30), Degrees.of(50));

        realNumberSet.add(interval1);
        realNumberSet.add(interval2);
        realNumberSet.add(interval3);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(1, intervals.size());
        assertEquals(Degrees.of(10), intervals.get(0).getStart());
        assertEquals(Degrees.of(50), intervals.get(0).getEnd());
    }

    @Test
    public void testAddIntervalEnclosingExisting() {
        Interval<AngleUnit, Angle> interval1 = new Interval<>(Degrees.of(20), Degrees.of(30));
        Interval<AngleUnit, Angle> enclosingInterval = new Interval<>(Degrees.of(10), Degrees.of(40));

        realNumberSet.add(interval1);
        realNumberSet.add(enclosingInterval);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(1, intervals.size());
        assertEquals(Degrees.of(10), intervals.get(0).getStart());
        assertEquals(Degrees.of(40), intervals.get(0).getEnd());
    }

    @Test
    public void testAddIntervalBridgingTwoExistingIntervals() {
        Interval<AngleUnit, Angle> interval1 = new Interval<>(Degrees.of(10), Degrees.of(20));
        Interval<AngleUnit, Angle> interval2 = new Interval<>(Degrees.of(30), Degrees.of(40));
        Interval<AngleUnit, Angle> bridgingInterval = new Interval<>(Degrees.of(19), Degrees.of(31));

        realNumberSet.add(interval1);
        realNumberSet.add(interval2);
        realNumberSet.add(bridgingInterval);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(1, intervals.size());
        assertEquals(Degrees.of(10), intervals.get(0).getStart());
        assertEquals(Degrees.of(40), intervals.get(0).getEnd());
    }

    @Test
    public void testAddIdenticalIntervals() {
        Interval<AngleUnit, Angle> interval = new Interval<>(Degrees.of(15), Degrees.of(25));

        realNumberSet.add(interval);
        realNumberSet.add(interval);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(1, intervals.size());
        assertEquals(interval, intervals.get(0));
    }

    @Test
    public void testAddZeroWidthInterval() {
        Interval<AngleUnit, Angle> pointInterval = new Interval<>(Degrees.of(20), Degrees.of(20));

        realNumberSet.add(pointInterval);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(1, intervals.size());
        assertEquals(Degrees.of(20), intervals.get(0).getStart());
        assertEquals(Degrees.of(20), intervals.get(0).getEnd());
    }

    @Test
    public void testAddIntervalsInDescendingOrder() {
        Interval<AngleUnit, Angle> interval1 = new Interval<>(Degrees.of(40), Degrees.of(50));
        Interval<AngleUnit, Angle> interval2 = new Interval<>(Degrees.of(20), Degrees.of(30));

        realNumberSet.add(interval1);
        realNumberSet.add(interval2);

        List<Interval<AngleUnit, Angle>> intervals = realNumberSet.getIntervals();
        assertTrue(isValidIntervals(realNumberSet));
        assertEquals(2, intervals.size());
        assertEquals(Degrees.of(20), intervals.get(0).getStart());
        assertEquals(Degrees.of(30), intervals.get(0).getEnd());
        assertEquals(Degrees.of(40), intervals.get(1).getStart());
        assertEquals(Degrees.of(50), intervals.get(1).getEnd());
    }
}

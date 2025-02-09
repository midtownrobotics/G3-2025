package constraints.set;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.Interval;
import frc.robot.subsystems.superstructure.Constraints.RealNumberSet;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestDifference extends TestSet {
    private RealNumberSet<AngleUnit, Angle> set1;
    private RealNumberSet<AngleUnit, Angle> set2;

    @BeforeEach
    void setUp() {
        set1 = new RealNumberSet<>();
        set2 = new RealNumberSet<>();
    }

    @Test
    public void testNonOverlappingSets() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(20)));
        set2.add(new Interval<>(Degrees.of(30), Degrees.of(40)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.difference(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.difference(set1);

        assertTrue(isValidIntervals(result1));
        assertTrue(isValidIntervals(result2));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(1, result2.getIntervals().size());
        assertEquals(Degrees.of(10), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(20), result1.getIntervals().get(0).getEnd());
        assertEquals(Degrees.of(30), result2.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(40), result2.getIntervals().get(0).getEnd());
    }

    @Test
    public void testFullyOverlappingIntervals() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(10), Degrees.of(30)));

        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(0, result.getIntervals().size()); // Should be empty
    }

    @Test
    public void testPartialOverlapAtStart() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(10), Degrees.of(20)));

        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(1, result.getIntervals().size());
        assertEquals(Degrees.of(20), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(30), result.getIntervals().get(0).getEnd());
    }

    @Test
    public void testPartialOverlapAtEnd() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(20), Degrees.of(30)));

        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(1, result.getIntervals().size());
        assertEquals(Degrees.of(10), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(20), result.getIntervals().get(0).getEnd());
    }

    @Test
    public void testIntervalInsideAnother() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(15), Degrees.of(25)));

        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(2, result.getIntervals().size());
        assertEquals(Degrees.of(10), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(15), result.getIntervals().get(0).getEnd());
        assertEquals(Degrees.of(25), result.getIntervals().get(1).getStart());
        assertEquals(Degrees.of(30), result.getIntervals().get(1).getEnd());
    }

    @Test
    public void testMultipleOverlappingIntervals() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(20)));
        set1.add(new Interval<>(Degrees.of(30), Degrees.of(40)));
        set2.add(new Interval<>(Degrees.of(15), Degrees.of(35)));

        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(2, result.getIntervals().size());
        assertEquals(Degrees.of(10), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(15), result.getIntervals().get(0).getEnd());
        assertEquals(Degrees.of(35), result.getIntervals().get(1).getStart());
        assertEquals(Degrees.of(40), result.getIntervals().get(1).getEnd());
    }

    @Test
    public void testOneSetIsSubsetOfAnother() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(50)));
        set2.add(new Interval<>(Degrees.of(20), Degrees.of(30)));

        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(2, result.getIntervals().size());
        assertEquals(Degrees.of(10), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(20), result.getIntervals().get(0).getEnd());
        assertEquals(Degrees.of(30), result.getIntervals().get(1).getStart());
        assertEquals(Degrees.of(50), result.getIntervals().get(1).getEnd());
    }

    @Test
    public void testOneSetEmpty() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(20)));

        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(1, result.getIntervals().size());
        assertTrue(isEquivalentSets(result, set1)); // The difference should be the original set
    }

    @Test
    public void testBothSetsEmpty() {
        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(0, result.getIntervals().size());
    }

    @Test
    public void testRemovingEverythingLeavesEmptySet() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(50)));
        set2.add(new Interval<>(Degrees.of(5), Degrees.of(55)));

        RealNumberSet<AngleUnit, Angle> result = set1.difference(set2);

        assertTrue(isValidIntervals(result));
        assertEquals(0, result.getIntervals().size()); // Should be empty
    }
}

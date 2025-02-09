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

public class TestUnion extends TestSet {
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

        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(2, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testFullyOverlappingIntervals() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(10), Degrees.of(30)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(10), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(30), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testOverlappingIntervalsMerged() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(25)));
        set2.add(new Interval<>(Degrees.of(20), Degrees.of(30)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(10), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(30), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testAdjacentIntervalsMerged() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(20)));
        set2.add(new Interval<>(Degrees.of(20), Degrees.of(30)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(10), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(30), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testMultipleOverlappingIntervals() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(20)));
        set1.add(new Interval<>(Degrees.of(30), Degrees.of(40)));
        set2.add(new Interval<>(Degrees.of(15), Degrees.of(35)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(10), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(40), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testMultipleDisjointIntervals() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(15)));
        set1.add(new Interval<>(Degrees.of(25), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(20), Degrees.of(22)));
        set2.add(new Interval<>(Degrees.of(35), Degrees.of(40)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(4, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testSubsetUnion() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(50)));
        set2.add(new Interval<>(Degrees.of(20), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(40), Degrees.of(45)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(10), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(50), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testOneSetEmpty() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(20)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testBothSetsEmpty() {
        RealNumberSet<AngleUnit, Angle> result1 = set1.union(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.union(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(0, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }
}

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

public class TestIntersect extends TestSet {
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

        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(0, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testFullyOverlappingIntervals() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(10), Degrees.of(30)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        System.out.println(result1);
        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(10), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(30), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testPartialOverlapAtStart() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(5), Degrees.of(15)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(10), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(15), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testPartialOverlapAtEnd() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(20), Degrees.of(40)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(20), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(30), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testIntervalInsideAnother() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(15), Degrees.of(25)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(1, result1.getIntervals().size());
        assertEquals(Degrees.of(15), result1.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(25), result1.getIntervals().get(0).getEnd());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testMultipleOverlappingIntervals() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(20)));
        set1.add(new Interval<>(Degrees.of(30), Degrees.of(40)));
        set2.add(new Interval<>(Degrees.of(15), Degrees.of(35)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(2, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
public void testMultipleDisjointIntervals() {
    set1.add(new Interval<>(Degrees.of(10), Degrees.of(15)));
    set1.add(new Interval<>(Degrees.of(20), Degrees.of(25)));
    set1.add(new Interval<>(Degrees.of(30), Degrees.of(35)));

    set2.add(new Interval<>(Degrees.of(12), Degrees.of(22)));
    set2.add(new Interval<>(Degrees.of(32), Degrees.of(40)));

    RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
    RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

    assertTrue(isValidIntervals(result1));
    assertEquals(3, result1.getIntervals().size());
    assertEquals(Degrees.of(12), result1.getIntervals().get(0).getStart());
    assertEquals(Degrees.of(15), result1.getIntervals().get(0).getEnd());

    assertEquals(Degrees.of(20), result1.getIntervals().get(1).getStart());
    assertEquals(Degrees.of(22), result1.getIntervals().get(1).getEnd());

    assertEquals(Degrees.of(32), result1.getIntervals().get(2).getStart());
    assertEquals(Degrees.of(35), result1.getIntervals().get(2).getEnd());

    assertTrue(isEquivalentSets(result1, result2));
}


    @Test
    public void testSubsetIntersection() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(50)));
        set2.add(new Interval<>(Degrees.of(20), Degrees.of(30)));
        set2.add(new Interval<>(Degrees.of(40), Degrees.of(45)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(2, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testNoIntervalsInOneSet() {
        set1.add(new Interval<>(Degrees.of(10), Degrees.of(20)));

        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(0, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }

    @Test
    public void testBothSetsEmpty() {
        RealNumberSet<AngleUnit, Angle> result1 = set1.intersection(set2);
        RealNumberSet<AngleUnit, Angle> result2 = set2.intersection(set1);

        assertTrue(isValidIntervals(result1));
        assertEquals(0, result1.getIntervals().size());
        assertTrue(isEquivalentSets(result1, result2));
    }
}

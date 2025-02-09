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

public class TestComplement extends TestSet {
    private RealNumberSet<AngleUnit, Angle> set;

    @BeforeEach
    void setUp() {
        set = new RealNumberSet<>();
    }

    @Test
    public void testComplementOfEmptySet() {
        RealNumberSet<AngleUnit, Angle> result = set.complement(Degrees.of(0), Degrees.of(100));

        assertTrue(isValidIntervals(result));
        assertEquals(1, result.getIntervals().size());
        assertEquals(Degrees.of(0), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(100), result.getIntervals().get(0).getEnd());
    }

    @Test
    public void testComplementWithinBounds() {
        set.add(new Interval<>(Degrees.of(20), Degrees.of(40)));

        RealNumberSet<AngleUnit, Angle> result = set.complement(Degrees.of(0), Degrees.of(100));

        assertTrue(isValidIntervals(result));
        assertEquals(2, result.getIntervals().size());

        assertEquals(Degrees.of(0), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(20), result.getIntervals().get(0).getEnd());

        assertEquals(Degrees.of(40), result.getIntervals().get(1).getStart());
        assertEquals(Degrees.of(100), result.getIntervals().get(1).getEnd());
    }

    @Test
    public void testComplementWithMultipleIntervals() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(20)));
        set.add(new Interval<>(Degrees.of(40), Degrees.of(50)));

        RealNumberSet<AngleUnit, Angle> result = set.complement(Degrees.of(0), Degrees.of(60));

        assertTrue(isValidIntervals(result));
        assertEquals(3, result.getIntervals().size());

        assertEquals(Degrees.of(0), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(10), result.getIntervals().get(0).getEnd());

        assertEquals(Degrees.of(20), result.getIntervals().get(1).getStart());
        assertEquals(Degrees.of(40), result.getIntervals().get(1).getEnd());

        assertEquals(Degrees.of(50), result.getIntervals().get(2).getStart());
        assertEquals(Degrees.of(60), result.getIntervals().get(2).getEnd());
    }

    @Test
    public void testComplementWhenSetCoversBounds() {
        set.add(new Interval<>(Degrees.of(0), Degrees.of(100)));

        RealNumberSet<AngleUnit, Angle> result = set.complement(Degrees.of(0), Degrees.of(100));

        assertTrue(isValidIntervals(result));
        assertEquals(0, result.getIntervals().size()); // Complement should be empty
    }

    @Test
    public void testComplementWhenIntervalsTouchBounds() {
        set.add(new Interval<>(Degrees.of(0), Degrees.of(50)));

        RealNumberSet<AngleUnit, Angle> result = set.complement(Degrees.of(0), Degrees.of(100));

        assertTrue(isValidIntervals(result));
        assertEquals(1, result.getIntervals().size());

        assertEquals(Degrees.of(50), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(100), result.getIntervals().get(0).getEnd());
    }

    @Test
    public void testComplementWithDisjointIntervals() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(30)));
        set.add(new Interval<>(Degrees.of(50), Degrees.of(70)));
        set.add(new Interval<>(Degrees.of(80), Degrees.of(90)));

        RealNumberSet<AngleUnit, Angle> result = set.complement(Degrees.of(0), Degrees.of(100));

        assertTrue(isValidIntervals(result));
        assertEquals(4, result.getIntervals().size());

        assertEquals(Degrees.of(0), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(10), result.getIntervals().get(0).getEnd());

        assertEquals(Degrees.of(30), result.getIntervals().get(1).getStart());
        assertEquals(Degrees.of(50), result.getIntervals().get(1).getEnd());

        assertEquals(Degrees.of(70), result.getIntervals().get(2).getStart());
        assertEquals(Degrees.of(80), result.getIntervals().get(2).getEnd());

        assertEquals(Degrees.of(90), result.getIntervals().get(3).getStart());
        assertEquals(Degrees.of(100), result.getIntervals().get(3).getEnd());
    }

    @Test
    public void testComplementWithLowerBoundAboveSet() {
        set.add(new Interval<>(Degrees.of(10), Degrees.of(30)));

        RealNumberSet<AngleUnit, Angle> result = set.complement(Degrees.of(20), Degrees.of(50));

        assertTrue(isValidIntervals(result));
        assertEquals(1, result.getIntervals().size()); // 🔥 Corrected from 2 to 1

        assertEquals(Degrees.of(30), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(50), result.getIntervals().get(0).getEnd());
    }


    @Test
    public void testComplementWithUpperBoundBelowSet() {
        set.add(new Interval<>(Degrees.of(40), Degrees.of(80)));

        RealNumberSet<AngleUnit, Angle> result = set.complement(Degrees.of(0), Degrees.of(50));

        assertTrue(isValidIntervals(result));
        assertEquals(1, result.getIntervals().size());

        assertEquals(Degrees.of(0), result.getIntervals().get(0).getStart());
        assertEquals(Degrees.of(40), result.getIntervals().get(0).getEnd());
    }
}

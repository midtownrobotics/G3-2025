// package constraints.tests;
// import static edu.wpi.first.units.Units.Radians;
// import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.junit.jupiter.api.Assertions.assertFalse;
// import static org.junit.jupiter.api.Assertions.assertTrue;

// import constraints.RealNumberTestSet;
// import edu.wpi.first.units.AngleUnit;
// import edu.wpi.first.units.measure.Angle;
// import frc.robot.subsystems.superstructure.Constraints.Interval;
// import org.junit.jupiter.api.Test;

// public class TestSet {

//     private RealNumberTestSet emptySet() {
//         return new RealNumberTestSet();
//     }

//     private Angle d(double num) {
//         return Radians.of(num);
//     }

//     @Test
//     public void testEmpty() {
//         RealNumberTestSet set = emptySet();

//         assertEquals(set.getIntervals().size(), 0);
//         assertFalse(set.containsNumber(0));
//         assertTrue(isValidIntervals(set));
//     }

//     @Test
//     public void testPoint() {
//         RealNumberTestSet set = emptySet();

//         set.add(new Interval<AngleUnit,Angle>(d(0), d(0)));
//         assertEquals(set.getIntervals().size(), 1);
//         assertTrue(set.containsNumber(0));
//         assertFalse(set.containsNumber(1));
//         assertTrue(isValidIntervals(set));
//     }

//     @Test
//     public void testRange() {
//         RealNumberTestSet set = emptySet();

//         set.add(new Interval<AngleUnit,Angle>(d(0), d(1)));

//         assertEquals(set.getIntervals().size(), 1);
//         assertTrue(set.containsNumber(0));
//         assertTrue(set.containsNumber(0.5));
//         assertTrue(set.containsNumber(1));
//         assertFalse(set.containsNumber(1.5));
//         assertTrue(isValidIntervals(set));
//     }

//     @Test
//     public void testAdd1() {
//         RealNumberTestSet set = emptySet();

//         set.add(new Interval<AngleUnit,Angle>(d(0), d(1)));
//         set.add(new Interval<AngleUnit,Angle>(d(2), d(3)));

//         assertEquals(set.getIntervals().size(), 2);
//         assertTrue(set.containsNumber(0));
//         assertTrue(set.containsNumber(0.5));
//         assertTrue(set.containsNumber(1));
//         assertFalse(set.containsNumber(1.5));
//         assertTrue(set.containsNumber(2.5));
//         assertTrue(isValidIntervals(set));
//     }

//     @Test
//     public void testAdd2() {
//         RealNumberTestSet set = emptySet();

//         set.add(new Interval<AngleUnit,Angle>(d(0), d(1)));
//         set.add(new Interval<AngleUnit,Angle>(d(2), d(3)));

//         set.add(new Interval<AngleUnit,Angle>(d(0.5), d(1.5)));

//         assertEquals(set.getIntervals().size(), 2);
//         assertTrue(set.containsNumber(0));
//         assertTrue(set.containsNumber(0.5));
//         assertTrue(set.containsNumber(1.5));
//         assertFalse(set.containsNumber(1.75));
//         assertTrue(set.containsNumber(2.5));
//         assertTrue(isValidIntervals(set));
//     }

//     @Test
//     public void testAdd3() {
//         RealNumberTestSet set = emptySet();

//         set.add(new Interval<AngleUnit,Angle>(d(0), d(1)));
//         set.add(new Interval<AngleUnit,Angle>(d(2), d(3)));

//         set.add(new Interval<AngleUnit,Angle>(d(0.5), d(1.5)));

//         set.add(new Interval<AngleUnit,Angle>(d(1.5), d(2)));

//         assertEquals(set.getIntervals().size(), 1);
//         assertTrue(set.containsNumber(0));
//         assertTrue(set.containsNumber(0.5));
//         assertTrue(set.containsNumber(1.5));
//         assertTrue(set.containsNumber(1.75));
//         assertTrue(set.containsNumber(2.5));
//         assertTrue(isValidIntervals(set));
//     }

//     @Test
//     public void testIntersectNone() {
//         RealNumberTestSet set1 = emptySet();

//         set1.add(new Interval<AngleUnit,Angle>(d(0), d(1)));

//         RealNumberTestSet set2 = emptySet();

//         set2.add(new Interval<AngleUnit,Angle>(d(2), d(3)));

//         RealNumberTestSet result = set1.intersection(set2);
//         RealNumberTestSet result2 = set2.intersection(set1);

//         assertEquals(result.toString(), result2.toString());

//         assertEquals(result.getIntervals().size(), 0);
//         assertTrue(isValidIntervals(result));
//     }

//     @Test
//     public void testIntersectPoint() {
//         RealNumberTestSet set1 = emptySet();

//         set1.add(new Interval<AngleUnit,Angle>(d(0), d(1)));

//         RealNumberTestSet set2 = emptySet();

//         set2.add(new Interval<AngleUnit,Angle>(d(1), d(2)));

//         RealNumberTestSet result = set1.intersection(set2);
//         RealNumberTestSet result2 = set2.intersection(set1);

//         assertEquals(result.toString(), result2.toString());

//         assertEquals(result.getIntervals().size(), 1);
//         assertTrue(result.containsNumber(1));
//         assertFalse(result.containsNumber(0.5));
//         assertTrue(isValidIntervals(result));
//     }

//     @Test
//     public void testIntersectRange() {
//         RealNumberTestSet set1 = emptySet();

//         set1.add(new Interval<AngleUnit,Angle>(d(0), d(2)));

//         RealNumberTestSet set2 = emptySet();

//         set2.add(new Interval<AngleUnit,Angle>(d(1), d(3)));

//         RealNumberTestSet result = set1.intersection(set2);
//         RealNumberTestSet result2 = set2.intersection(set1);

//         assertEquals(result.toString(), result2.toString());

//         assertEquals(result.getIntervals().size(), 1);
//         assertTrue(result.containsNumber(1.5));
//         assertTrue(result.containsNumber(1));
//         assertFalse(result.containsNumber(0.5));
//         assertFalse(result.containsNumber(2.5));
//         assertTrue(isValidIntervals(result));
//     }

//     @Test
//     public void testIntersectComplex() {
//         RealNumberTestSet set1 = emptySet();

//         set1.add(new Interval<AngleUnit,Angle>(d(1), d(2)));
//         set1.add(new Interval<AngleUnit,Angle>(d(3), d(4)));
//         set1.add(new Interval<AngleUnit,Angle>(d(10), d(12)));

//         RealNumberTestSet set2 = emptySet();

//         set2.add(new Interval<AngleUnit,Angle>(d(1.5), d(3.5)));
//         set2.add(new Interval<AngleUnit,Angle>(d(4.5), d(8)));
//         set2.add(new Interval<AngleUnit,Angle>(d(9), d(12)));

//         RealNumberTestSet result = set1.intersection(set2);
//         RealNumberTestSet result2 = set2.intersection(set1);

//         assertEquals(result.toString(), result2.toString());

//         assertEquals(result.getIntervals().size(), 3);
//         assertTrue(result.containsNumber(1.75));
//         assertTrue(result.containsNumber(3.5));
//         assertTrue(result.containsNumber(10));
//         assertTrue(result.containsNumber(12));

//         assertFalse(result.containsNumber(1.2));
//         assertFalse(result.containsNumber(2.5));
//         assertFalse(result.containsNumber(3.7));
//         assertFalse(result.containsNumber(5));
//         assertFalse(result.containsNumber(7));
//         assertFalse(result.containsNumber(9.5));

//         assertTrue(isValidIntervals(result));
//     }

//     @Test
//     public void testDifferenceNone() {
//         RealNumberTestSet set1 = emptySet();

//         set1.add(new Interval<AngleUnit,Angle>(d(0), d(1)));

//         RealNumberTestSet set2 = emptySet();

//         set2.add(new Interval<AngleUnit,Angle>(d(2), d(3)));

//         RealNumberTestSet result = set1.difference(set2);

//         assertEquals(result.toString(), set1.toString());

//         assertEquals(result.getIntervals().size(), 1);
//         assertTrue(isValidIntervals(result));
//     }

//     @Test
//     public void testDifferencePoint() {
//         RealNumberTestSet set1 = emptySet();

//         set1.add(new Interval<AngleUnit,Angle>(d(0), d(1)));

//         RealNumberTestSet set2 = emptySet();

//         set2.add(new Interval<AngleUnit,Angle>(d(1), d(2)));

//         RealNumberTestSet result = set1.difference(set2);

//         assertEquals(result.toString(), set1.toString());

//         assertEquals(result.getIntervals().size(), 1);
//         assertTrue(isValidIntervals(result));
//     }

//     @Test
//     public void testDifferenceRange() {
//         RealNumberTestSet set1 = emptySet();

//         set1.add(new Interval<AngleUnit,Angle>(d(0), d(1)));

//         RealNumberTestSet set2 = emptySet();

//         set2.add(new Interval<AngleUnit,Angle>(d(0.5), d(1.5)));

//         RealNumberTestSet result = set1.difference(set2);

//         assertEquals(result.getIntervals().size(), 1);
//         assertTrue(result.containsNumber(0.25));
//         assertFalse(result.containsNumber(0.75));
//         assertTrue(isValidIntervals(result));
//     }

//     @Test
//     public void testDifferenceComplex() {
//         RealNumberTestSet set1 = emptySet();

//         set1.add(new Interval<AngleUnit,Angle>(d(1), d(2)));
//         set1.add(new Interval<AngleUnit,Angle>(d(3), d(4)));
//         set1.add(new Interval<AngleUnit,Angle>(d(10), d(12)));

//         RealNumberTestSet set2 = emptySet();

//         set2.add(new Interval<AngleUnit,Angle>(d(1.5), d(3.5)));
//         set2.add(new Interval<AngleUnit,Angle>(d(4.5), d(8)));
//         set2.add(new Interval<AngleUnit,Angle>(d(9), d(12)));

//         RealNumberTestSet result = set1.difference(set2);

//         assertEquals(result.getIntervals().size(), 2);
//         assertTrue(result.containsNumber(1.25));
//         assertTrue(result.containsNumber(3.75));

//         assertFalse(result.containsNumber(1.6));
//         assertFalse(result.containsNumber(2.5));
//         assertFalse(result.containsNumber(3.3));
//         assertFalse(result.containsNumber(5));
//         assertFalse(result.containsNumber(7));
//         assertFalse(result.containsNumber(9));
//         assertFalse(result.containsNumber(11));

//         assertTrue(isValidIntervals(result));
//     }


//     private boolean isValidIntervals(RealNumberTestSet set) {
//         double previous = -Double.MAX_VALUE;

//         for (Interval<AngleUnit, Angle> interval : set.getIntervals()) {
//             double start = interval.getStart().baseUnitMagnitude();

//             if (start <= previous) {
//                 return false;
//             }

//             double end = interval.getEnd().baseUnitMagnitude();

//             if (end < start) {
//                 return false;
//             }

//             previous = end;
//         }

//         return true;
//     }

// }

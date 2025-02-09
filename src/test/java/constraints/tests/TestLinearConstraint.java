package constraints.tests;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import org.junit.jupiter.api.Test;

public class TestLinearConstraint {

    /** Makes angle from double */
    private Angle ma(double degrees) {
        return Degrees.of(degrees);
    }

    @Test
    public void testBasicStayIn() {
        CircularConstraint constraint = new CircularConstraint();

        constraint.addStayInConstraint(ma(20), ma(60));

        assertEquals(constraint.getClosestToDesired(ma(45), ma(90)), ma(60));
        assertEquals(constraint.getClosestToDesired(ma(45), ma(15)), ma(20));
        assertEquals(constraint.getClosestToDesired(ma(45), ma(40)), ma(40));
    }

    @Test
    public void testWrapAroundStayIn() {
        CircularConstraint constraint = new CircularConstraint();

        // constraint.addKeepOutConstraint(ma(60), ma(300));

        constraint.addStayInConstraint(ma(300), ma(60));

        assertEquals(constraint.getClosestToDesired(ma(60), ma(315)), ma(-45));
        assertEquals(constraint.getClosestToDesired(ma(300), ma(45)), ma(405));
        assertEquals(constraint.getClosestToDesired(ma(320), ma(80)), ma(420));
        assertEquals(constraint.getClosestToDesired(ma(320), ma(200)), ma(300));
        assertEquals(constraint.getClosestToDesired(ma(20), ma(280)), ma(-60));
        assertEquals(constraint.getClosestToDesired(ma(20), ma(160)), ma(60));
    }

    @Test
    public void testBasicKeepOut() {
        CircularConstraint constraint = new CircularConstraint();

        constraint.addKeepOutConstraint(ma(20), ma(100));

        assertEquals(constraint.getClosestToDesired(null, null), constraint);

        // assertEquals(constraint.getClosestToDesired(ma(0), ma(120)), ma(-240));
        // assertEquals(constraint.getClosestToDesired(ma, null), constraint);
    }
}

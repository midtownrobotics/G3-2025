package frc.robot.subsystems.superstructure.Constraints;

import static frc.robot.utils.UnitUtil.clamp;
import static frc.robot.utils.UnitUtil.max;
import static frc.robot.utils.UnitUtil.min;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class LinearConstraint<U extends Unit, M extends Measure<U>> {
    private M lower;
    private M upper;

    /**
     * Constructor for a new Linear Constraint
     */
    public LinearConstraint(M lower, M upper) {
        this.lower = lower;
        this.upper = upper;
    }

    /** Returns a value within the set bounds */
    public M getClampedValue(M value) {
        return clamp(value, lower, upper);
    }

    /** Puts upper and lower constraints at the given value */
    public void restrictToValue(M value) {
        lower = value;
        upper = value;
    }
}

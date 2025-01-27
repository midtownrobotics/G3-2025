package frc.robot.subsystems.superstructure.Constraints;

import edu.wpi.first.units.Measure;

public class StayInConstraint<T extends Measure<?>> implements Constraint<T> {

    private T min;
    private T max;

    /** NO LONGER NEEDED */
    public StayInConstraint(T min, T max) {
        this.min = min;
        this.max = max;
    }

    @Override
    public T apply(T target, T current) {
        if (current.baseUnitMagnitude() < min.baseUnitMagnitude() || current.baseUnitMagnitude() > max.baseUnitMagnitude()) {
            return Math.abs(current.baseUnitMagnitude() - min.baseUnitMagnitude()) > Math.abs(current.baseUnitMagnitude() - max.baseUnitMagnitude()) ? max : min;
        }
        if (current.baseUnitMagnitude() < max.baseUnitMagnitude() && target.baseUnitMagnitude() > max.baseUnitMagnitude()) {
            return max;
        }
        if (current.baseUnitMagnitude() > min.baseUnitMagnitude() && target.baseUnitMagnitude() < min.baseUnitMagnitude()) {
            return min;
        }
        return target;
    }

}

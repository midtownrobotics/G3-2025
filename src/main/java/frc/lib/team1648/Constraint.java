package frc.lib.team1648;

import edu.wpi.first.units.Measure;
import lombok.Getter;

public class Constraint<T extends Measure<?>> {
    @Getter
    T min;
    @Getter
    T max;

    /** Constructor for contraints */
    public Constraint(T min, T max) {
        this.min = min;
        this.max = max;
    }

    /** Clamps the inputted value */
    public T clamp(T target, T current) {
        if (current.baseUnitMagnitude() > max.baseUnitMagnitude() && target.baseUnitMagnitude() < max.baseUnitMagnitude()) {
            return max;
        }
        if (current.baseUnitMagnitude() < min.baseUnitMagnitude() && target.baseUnitMagnitude() > min.baseUnitMagnitude()) {
            return min;
        }
        if (current.baseUnitMagnitude() > min.baseUnitMagnitude() && current.baseUnitMagnitude() < max.baseUnitMagnitude()) {
            return Math.abs(current.baseUnitMagnitude() - min.baseUnitMagnitude()) > Math.abs(current.baseUnitMagnitude() - max.baseUnitMagnitude()) ? max : min;
        }
        return target;

    }
}

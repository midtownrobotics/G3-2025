package frc.lib.team1648;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import lombok.Getter;

public class Constraint<T extends Measure<?>> {
    @Getter
    T min;
    @Getter
    T max;

    public Constraint(T min, T max) {
        this.min = min;
        this.max = max;
    }

    public T clamp(T value) {
        if (value.baseUnitMagnitude() > max.baseUnitMagnitude()) {
            return max;
        }
        if (value.baseUnitMagnitude() < min.baseUnitMagnitude()) {
            return min;
        }
        return value;

    }
}

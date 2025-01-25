package frc.lib.team1648;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import lombok.Getter;

public class Constraints<T extends Measure<Unit>> {
    @Getter
    T min;
    @Getter
    T max;

    public Constraints(T min, T max) {
        this.min = min;
        this.max = max;
    }

    public T clamp(T value) {
        if (value.gt(max)) {
            return max;
        }
        if (value.lt(min)) {
            return min;
        }
        return value;

    }
}

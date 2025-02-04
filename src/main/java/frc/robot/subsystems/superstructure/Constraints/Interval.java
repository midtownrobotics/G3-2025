package frc.robot.subsystems.superstructure.Constraints;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import lombok.Getter;
import lombok.Setter;

public class Interval<U extends Unit, T extends Measure<U>> {
    @Getter @Setter private T start;
    @Getter @Setter private T end;

    /**
     * Constructs an Interval within a Set
     */
    public Interval(T start, T end) {
        if (start.gt(end)) throw new IllegalArgumentException("Start cannot be greater than end.");
        this.start = start;
        this.end = end;
    }

    @Override
    public String toString() {
        return "[" + start.baseUnitMagnitude() + ", " + end.baseUnitMagnitude() + ")";
    }
}

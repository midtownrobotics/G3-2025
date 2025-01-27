package frc.robot.subsystems.superstructure.Constraints;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class Interval<U extends Unit, T extends Measure<U>> implements Comparable<Interval<U, T>> {
    T start;
    T end;

    /**
     * Constructs an Interval within a Set
     */
    public Interval(T start, T end) {
        if (start.gt(end)) throw new IllegalArgumentException("Start cannot be greater than end.");
        this.start = start;
        this.end = end;
    }

    @Override
    public int compareTo(Interval<U, T> other) {
        // TODO: Double check this (?)
        return start.compareTo(other.start);
    }

    @Override
    public String toString() {
        return "[" + start.baseUnitMagnitude() + ", " + end.baseUnitMagnitude() + ")";
    }
}

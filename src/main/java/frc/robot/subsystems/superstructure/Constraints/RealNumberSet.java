package frc.robot.subsystems.superstructure.Constraints;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.ArrayList;
import java.util.List;

public class RealNumberSet<U extends Unit, T extends Measure<U>> {
    private List<Interval<U, T>> intervals;

    /**
     * Constructs a new RealNumberSet
     */
    public RealNumberSet() {
        this.intervals = new ArrayList<>();
    }

    /**
     * Adds and optimizes a new interval to the set
     */
    public RealNumberSet<U, T> add(Interval<U, T> interval) {
        int i = 0;

        while (i < intervals.size() && intervals.get(i).getEnd().lt(interval.getStart())) {
            i++;
        }

        while (i < intervals.size() && intervals.get(i).getStart().lte(interval.getEnd())) {
            Interval<U, T> removedInterval = intervals.remove(i);
            interval.setStart(min(interval.getStart(), removedInterval.getStart()));
            interval.setEnd(max(interval.getEnd(), removedInterval.getEnd()));
        }

        intervals.add(i, interval);

        return this;
    }

    /**
     * Returns the union of two sets (NOT IN PLACE)
     */
    public RealNumberSet<U, T> union(RealNumberSet<U, T> other) {
        
        RealNumberSet<U, T> result = new RealNumberSet<U, T>();
        int i = 0, j = 0;
        while (i < this.intervals.size() || j < other.intervals.size()) {
            Interval<U, T> next;
            if (j >= other.intervals.size() || (i < this.intervals.size() && this.intervals.get(i).getStart().lt(other.intervals.get(j).getEnd()))) {
                next = this.intervals.get(i++);
            } else {
                next = other.intervals.get(j++);
            }
            result.add(new Interval<>(next.getStart(), next.getEnd()));
        }
        return result;
    }

    /**
     * Intersects two sets (NOT IN PLACE)
     */
    public RealNumberSet<U, T> intersection(RealNumberSet<U, T> other) {
        RealNumberSet<U, T> result = new RealNumberSet<U, T>();
        int i = 0, j = 0;
        while (i < this.intervals.size() && j < other.intervals.size()) {
            Interval<U, T> a = this.intervals.get(i);
            Interval<U, T> b = other.intervals.get(j);

            // Find overlap
            T start = max(a.getStart(), b.getStart());
            T end = min(a.getEnd(), b.getEnd());

            if (start.lt(end)) { // Overlapping
                result.add(new Interval<>(start, end));
            }

            // Move to the next interval
            if (a.getEnd().lt(b.getEnd())) {
                i++;
            } else {
                j++;
            }
        }
        return result;
    }

    /**
     * Subtracts the given set from this set. NOT INPLACE
     */
    public RealNumberSet<U, T> difference(RealNumberSet<U, T> other) {
        RealNumberSet<U, T> result = new RealNumberSet<U, T>();
        for (Interval<U, T> a : this.intervals) {
            T currentStart = a.getStart();
            T currentEnd = a.getEnd();
            for (Interval<U, T> b : other.intervals) {
                if (b.getEnd().lte(currentStart)) continue;
                if (b.getStart().gte(currentEnd)) break;
                if (b.getStart().gt(currentStart)) {
                    result.add(new Interval<>(currentStart, min(b.getStart(), currentEnd)));
                }
                currentStart = max(currentStart, b.getEnd());
                if (currentStart.gte(currentEnd)) break;
            }
            if (currentStart.lt(currentEnd)) {
                result.add(new Interval<>(currentStart, currentEnd));
            }
        }
        return result;
    }

    @Override
    public String toString() {
        return intervals.toString();
    }

    private T max(T a, T b) {
        if (a.gt(b)) return a;
        return b;
    }

    private T min (T a, T b) {
        if (a.lt(b)) return a;
        return b;
    }

    /**
     * Returns the interval that contains the given value
     */
    public Interval<U, T> getIntervalOfValue(T value) {
        for (Interval<U, T> interval : intervals) {
            if (value.lt(interval.getStart())) continue;
            if (value.lte(interval.getEnd())) {
                return interval;
            }
            break;
        }
        return null;
    }
}

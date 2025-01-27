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
    public void add(T start, T end) {
        Interval<U, T> newInterval = new Interval<>(start, end);
        List<Interval<U, T>> result = new ArrayList<>();
        int i = 0;

        // Add all intervals ending before the new interval starts
        while (i < intervals.size() && intervals.get(i).end.lt(newInterval.start)) {
            result.add(intervals.get(i++));
        }

        // Merge overlapping intervals
        while (i < intervals.size() && intervals.get(i).start.lte(newInterval.end)) {
            newInterval.start = min(newInterval.start, intervals.get(i).start);
            newInterval.end = max(newInterval.end, intervals.get(i).end);
            i++;
        }
        result.add(newInterval);

        // Add remaining intervals
        while (i < intervals.size()) {
            result.add(intervals.get(i++));
        }

        this.intervals = result;
    }

    /**
     * Returns the union of two sets (NOT IN PLACE)
     */
    public RealNumberSet<U, T> union(RealNumberSet<U, T> other) {
        RealNumberSet<U, T> result = new RealNumberSet<U, T>();
        int i = 0, j = 0;
        while (i < this.intervals.size() || j < other.intervals.size()) {
            Interval<U, T> next;
            if (j >= other.intervals.size() || (i < this.intervals.size() && this.intervals.get(i).start.lt(other.intervals.get(j).start))) {
                next = this.intervals.get(i++);
            } else {
                next = other.intervals.get(j++);
            }
            result.add(next.start, next.end);
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
            T start = max(a.start, b.start);
            T end = min(a.end, b.end);

            if (start.lt(end)) { // Overlapping
                result.add(start, end);
            }

            // Move to the next interval
            if (a.end.lt(b.end)) {
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
            T currentStart = a.start;
            T currentEnd = a.end;
            for (Interval<U, T> b : other.intervals) {
                if (b.end.lte(currentStart)) continue;
                if (b.start.gte(currentEnd)) break;
                if (b.start.gt(currentStart)) {
                    result.add(currentStart, min(b.start, currentEnd));
                }
                currentStart = max(currentStart, b.end);
                if (currentStart.gte(currentEnd)) break;
            }
            if (currentStart.lt(currentEnd)) {
                result.add(currentStart, currentEnd);
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
            if (value.lt(interval.start)) continue;
            if (value.lte(interval.end)) {
                return interval;
            }
            break;
        }
        return null;
    }
}

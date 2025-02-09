package frc.robot.utils;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class UnitUtil {

    /**
     * Util which returns the max of two measures
     */
    public static <U extends Unit, T extends Measure<U>> T max(T a, T b) {
        if (a.gt(b)) return a;
        return b;
    }

    /**
     * Util which returns the min of two measures
     */
    public static <U extends Unit, T extends Measure<U>> T min (T a, T b) {
        if (a.lt(b)) return a;
        return b;
    }

}

package frc.robot.subsystems.superstructure.Constraints;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class ClawConstraint implements Constraint<Angle>{

    private Angle min;
    private Angle max;

    /**
     *
     * @param min
     * @param max
     */
    public ClawConstraint(Angle min, Angle max) {
        this.min = min;
        this.max = max;
    }

    /** if greater than 180, converts to negative angle */
    private double abs(double val) {
        return val <= 180 ? val : val - 360;
    }

    /** returns if current less than target */
    private boolean lt(Angle current, Angle target) {
        double targetDegrees = target.in(Units.Degrees);
        double currentDegrees = current.in(Units.Degrees);
        return abs(currentDegrees) < abs(targetDegrees);
    }

    @Override
    public Angle apply(Angle target, Angle current) {
        Angle error;
        if (target.minus(current).abs(Units.Degrees) < current.minus(target).abs(Units.Degrees)) {
            error = target.minus(current);
        } else {
            error = current.minus(target);
        }

        if (lt(current, min) && !lt(target, max)) {

        }

        return error;

    }

}

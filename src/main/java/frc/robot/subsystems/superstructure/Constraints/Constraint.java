package frc.robot.subsystems.superstructure.Constraints;

import edu.wpi.first.units.Measure;

public interface Constraint<T extends Measure<?>> {
    /** Applies the inputted value */
    public T apply(T target, T current);
}

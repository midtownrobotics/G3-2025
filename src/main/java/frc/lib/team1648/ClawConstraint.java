package frc.lib.team1648;

import edu.wpi.first.units.measure.Angle;

public class ClawConstraint extends Constraint<Angle>{

    public ClawConstraint(Angle min, Angle max) {
        super(min, max);
    }

    @Override
    public Angle clamp(Angle target, Angle current) {

        if (current.gt(max) && target.lt())

        return target;
    }

    public Angle clampHalf(Angle target, Angle current) {
        return target;
    }

    
    
}

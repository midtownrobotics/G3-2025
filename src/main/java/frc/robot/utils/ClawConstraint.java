package frc.robot.utils;

import frc.lib.team1648.Constraint;
import edu.wpi.first.units.measure.Angle;

public class ClawConstraint extends Constraint<Angle>{

    private final Angle absMin;
    private final Angle absMax;

    public ClawConstraint(Angle min, Angle max, Angle absMin, Angle absMax) {
        super(min, max);
        this.absMin = absMin;
        this.absMax = absMax;
    }

    @Override
    public Angle clamp(Angle target, Angle current) {
        // TODO Auto-generated method stub
        return super.clamp(value);
    }

    
    
}

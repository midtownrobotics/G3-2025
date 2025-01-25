package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class OuttakeUtils {
    
    public InterpolatingDoubleTreeMap lookupTableAngle;
    public static OuttakeUtils instance = new OuttakeUtils();

    public OuttakeUtils() {
    lookupTableAngle = new InterpolatingDoubleTreeMap();
    lookupTableAngle.put(159.4, 0.852);
    lookupTableAngle.put(181.0, 0.87);
    lookupTableAngle.put(202.0, 0.9);
    lookupTableAngle.put(220.0, 0.92);
    lookupTableAngle.put(240.0, 0.945); // 3800 RPM
    lookupTableAngle.put(260.0, 0.98);
    lookupTableAngle.put(280.0, 1.0);
    lookupTableAngle.put(300.0, 1.03);
    lookupTableAngle.put(320.0, 1.06);
    lookupTableAngle.put(340.0, 1.08);
    lookupTableAngle.put(360.0, 1.09);
    }

    public double getAngleFromDistance(double distance) {
        return lookupTableAngle.get(distance);
    }

    public AngularVelocity getSpeedFromDistance(double distance){
        if (distance >= 230) return Units.RPM.of(3800);
        return Constants.SPEAKER_SPEED; 
    }
}

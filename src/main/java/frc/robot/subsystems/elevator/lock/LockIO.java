package frc.robot.subsystems.elevator.lock;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface LockIO {
    @AutoLog
    public class LockInputs {
        public Angle position = Degrees.zero();
    }

    /** Set position */
    public void setPosition(Angle position);

    /** Update inputs */
    public void updateInputs(LockInputs inputs);
}

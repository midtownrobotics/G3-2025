package frc.robot.subsystems.elevator.lock;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface LockIO {
    /** Enable / disble lock. */
    public void setLockEnabled(boolean value);

    @AutoLog
    public class LockInputs {
        public boolean enabled = false;
        public Angle lastCommandedPosition = Degrees.zero();
    }

    /**
     * Updates input class for Winch
     *
     * @param inputs
     */
    public void updateInputs(LockInputs inputs);
}

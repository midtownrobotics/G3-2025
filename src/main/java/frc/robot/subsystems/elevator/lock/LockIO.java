package frc.robot.subsystems.elevator.lock;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface LockIO {
    @AutoLog
    public class LockInputs {
        public Angle position = Degrees.zero();
    }

    public void setPosition(Angle position);

    public void updateInputs(LockInputs inputs);
}

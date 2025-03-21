package frc.robot.subsystems.elevator.lock;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;

public class LockIORevServo implements LockIO{
    private final Servo servo;
    private static final Angle lockedPosition = Degrees.of(0), unlockedPosition = Degrees.of(108);

    /** Lock IO constructor for servo. */
    public LockIORevServo(int id) {
        servo = new Servo(id);
        servo.setBoundsMicroseconds(2500, 0, 1500, 0, 500);
    }

    /** Enable / disable lock. */
    public void setLockEnabled(boolean value) {
        double degrees = value ? lockedPosition.in(Degrees) : unlockedPosition.in(Degrees);

        if (degrees < 0) {
            degrees = 0;
          } else if (degrees > 270) {
            degrees = 270;
        }

        servo.set((270-degrees)/270);
    }
}

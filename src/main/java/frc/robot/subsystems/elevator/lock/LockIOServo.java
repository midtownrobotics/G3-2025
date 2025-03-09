package frc.robot.subsystems.elevator.lock;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;

public class LockIOServo implements LockIO {
    private final Servo servo;
    private static final Angle lockedPosition = Degrees.of(35), unlockedPosition = Degrees.of(75);

    /** Lock IO constructor for servo. */
    public LockIOServo(int id) {
        servo = new Servo(id);
        servo.setBoundsMicroseconds(2500, 0, 1500, 0, 500);
    }

    /** Enable / disble lock. */
    public void setLockEnabled(boolean value) {
        servo.setAngle(value ? lockedPosition.in(Degrees) : unlockedPosition.in(Degrees));
    }

}

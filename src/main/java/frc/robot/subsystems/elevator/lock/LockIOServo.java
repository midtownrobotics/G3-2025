package frc.robot.subsystems.elevator.lock;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;

public class LockIOServo implements LockIO {

    private final Servo servo;

    /** Servo lock IO */
    public LockIOServo(int servoID) {
        servo = new Servo(servoID);
    }

    @Override
    public void setPosition(Angle position) {
        servo.setAngle(position.in(Degrees));
    }

    @Override
    public void updateInputs(LockInputs inputs) {
        inputs.position = Degrees.of(servo.getAngle());
    }

}

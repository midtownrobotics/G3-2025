package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class Photoelectric {
    private DigitalInput sensor = new DigitalInput(0);

    public boolean isSensorTripped() {
        return sensor.get();
    }
}

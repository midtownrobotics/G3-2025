package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class Photoelectric {
    private DigitalInput sensor;

    /**
     * Constructor for a Photoelectric sensor
     * @param DIOPort
     */
    public Photoelectric(int DIOPort) {
        this.sensor = new DigitalInput(DIOPort);
    }

    public boolean isTriggered() {
        return sensor.get();
    }
}

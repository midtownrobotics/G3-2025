package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class PhotoElectric {
    private DigitalInput sensor;

    /**
     * Constructs a PhotoElectric Sensor
     */
    public PhotoElectric(int DIOPort) {
        this.sensor = new DigitalInput(DIOPort);
    }

    public boolean isTriggered() {
        return sensor.get();
    }
}

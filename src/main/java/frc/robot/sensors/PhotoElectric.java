package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class PhotoElectric {
    private DigitalInput sensor;

    public PhotoElectric(int DIOPort) {
        this.sensor = new DigitalInput(DIOPort);
    }

    public boolean isTriggered() {
        return sensor.get();
    }
}

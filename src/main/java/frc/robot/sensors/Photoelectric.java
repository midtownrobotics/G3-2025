package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.VirtualSubsystem;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Photoelectric extends VirtualSubsystem {
    private final DigitalInput sensor;
    private final int port;

    public class PhotoelectricInputs implements LoggableInputs {
        public boolean triggered;

        @Override
        public void toLog(LogTable table) {
            table.put("Triggered", triggered);
        }

        @Override
        public void fromLog(LogTable table) {
            triggered = table.get("Triggered", triggered);
        }
    }

    private final PhotoelectricInputs inputs = new PhotoelectricInputs();

    /**
     * Constructor for a Photoelectric sensor
     * @param DIOPort
     */
    public Photoelectric(int DIOPort) {
        this.port = DIOPort;
        this.sensor = new DigitalInput(DIOPort);
    }

    @Override
    public void periodic() {
        inputs.triggered = !sensor.get();
        Logger.processInputs("Photoelectric_" + port, inputs);
    }

    public boolean isTriggered() {
        return inputs.triggered;
    }


}

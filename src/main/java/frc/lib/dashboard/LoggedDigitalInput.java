package frc.lib.dashboard;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.VirtualSubsystem;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggedDigitalInput extends VirtualSubsystem {
    private final DigitalInput sensor;
    private final int port;

    public class DigitalInputInputs implements LoggableInputs {
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

    private final DigitalInputInputs inputs = new DigitalInputInputs();

    /**
     * Constructor for a logged digital input.
     * @param DIOPort
     */
    public LoggedDigitalInput(int DIOPort) {
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

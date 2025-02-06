package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.List;

public class CANBusStatusSignalRegistration {
    private List<StatusSignal<?>> signals= new ArrayList<>();

    /** registry for all status signals */
    public CANBusStatusSignalRegistration register(StatusSignal<?> signal) {
        signals.add(signal);
        return this;
    }

    /** refreshes all status signals */
    public void refreshSignals() {
        BaseStatusSignal.refreshAll(signals.toArray(BaseStatusSignal[]::new));
    }
}

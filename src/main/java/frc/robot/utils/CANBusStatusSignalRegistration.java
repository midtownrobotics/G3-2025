package frc.robot.utils;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class CANBusStatusSignalRegistration {
    List<StatusSignal<?>> signals;
    
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

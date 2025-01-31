package frc.lib.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class BatterySimManager {

    private static @Getter BatterySimManager instance;
    private Voltage nominalVoltage = Volts.of(13);
    private static final Resistance INTERNAL_RESISTANCE= Ohms.of(0.014);
    private static final double batteryCapacity = 8; // In Ah
    private double remainingCapacity = batteryCapacity;

    private final List<Supplier<Current>> loads = new ArrayList<>();

    private BatterySimManager() {}

    public static BatterySimManager createInstance() {
        instance = new BatterySimManager();
        RoboRioSim.setVInVoltage(instance.nominalVoltage.in(Volts));
        return instance;
    }

    public void registerSupplier(Supplier<Current> supplier) {
        this.loads.add(supplier);
    }

    public void periodic() {
        Current totalCurrent = Amps.of(3);

        for (Supplier<Current> supplier : loads) {
            totalCurrent = totalCurrent.plus(supplier.get());
        }

        Voltage loadVoltage = nominalVoltage.minus(INTERNAL_RESISTANCE.times(totalCurrent));
        RoboRioSim.setVInVoltage(loadVoltage.in(Volts));

        double capacityUsed = totalCurrent.in(Amps) * 0.02;
        remainingCapacity -= capacityUsed;

        double newRestingVoltage = calculateVoltage(remainingCapacity  /batteryCapacity);
        nominalVoltage = Volts.of(newRestingVoltage);

        Logger.recordOutput("BatterySim/restingVoltage", newRestingVoltage);
        Logger.recordOutput("BatterySim/loadVoltage", loadVoltage);
        Logger.recordOutput("BatterySim/remainingCapacity", remainingCapacity);
        Logger.recordOutput("BatterySim/totalCurrent", totalCurrent);
    }

    public Voltage getVoltage() {
        return Volts.of(RoboRioSim.getVInVoltage());
    }

    private double calculateVoltage(double soc) {
        soc = MathUtil.clamp(soc, 0, 1);
        if (soc >= 1) {
        return 13;
        } else if (soc > 0.5) {
        return 12 + (soc - 0.5) * 2;
        } else if (soc > 0.2) {
        return 11.5 + (soc - 0.2) * 0.5 / 0.3;
        } else if (soc > 0.05) {
        return 11.0 + (soc - 0.05) * 0.5 / 0.15;
        }
        return 10.5;
  }
}

package frc.robot.subsystems.coral_outtake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class CoralOuttakeConstants {
    public static final Voltage HANDOFF_VOLTAGE = Volts.of(6);
    public static final Voltage SHOOT_VOLTAGE = Volts.of(12);
    public static final Voltage STATION_INTAKE_VOLTAGE = Volts.of(6);

    public static final double HEAD_ON_ANGLE_ERROR = 5;
}

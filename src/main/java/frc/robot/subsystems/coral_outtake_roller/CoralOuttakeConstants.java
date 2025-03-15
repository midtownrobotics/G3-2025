package frc.robot.subsystems.coral_outtake_roller;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib.dashboard.LoggedTunableNumber;

public class CoralOuttakeConstants {
    public static final Angle headOnAngleError = Degrees.of(5);
    public static final Time coralDetectionIdleDelay = Milliseconds.of(100);
};

package frc.robot.utils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
  public static final boolean tuningMode = false;
  public static final Current KRAKEN_CURRENT_LIMIT = Units.Amp.of(70);
  public static final Current KRAKEN_CURRENT_LOWER_LIMIT = Units.Amp.of(40);
  public static final Current NEO_550_CURRENT_LIMIT = Units.Amp.of(35);
  public static final Current NEO_CURRENT_LIMIT = Units.Amp.of(60);
  public static final Current BAG_CURRENT_LIMIT = Units.Amp.of(70);
  public static final AngularVelocity SPEAKER_SPEED = Units.RPM.of(3100);
  public static final Angle SPEAKER_ANGLE = Units.Rotations.of(0.852);
  public static final double HEAD_ON_ANGLE_ERROR = 5;
  public static final Voltage SHOOT_VOLTAGE = Units.Volts.of(12);
  public static final Voltage HANDOFF_VOLTAGE = Units.Volts.of(0);
  public static final Voltage STATION_INTAKE_VOLTAGE = Units.Volts.of(0);
} 

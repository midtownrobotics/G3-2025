package frc.robot.utils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;

public class Constants {
  public static final boolean tuningMode = false;
  public static final Current KRAKEN_CURRENT_LIMIT = Units.Amp.of(70);
  public static final Current KRAKEN_CURRENT_LOWER_LIMIT = Units.Amp.of(40);
  public static final Current NEO_550_CURRENT_LIMIT = Units.Amp.of(35);
  public static final Current NEO_CURRENT_LIMIT = Units.Amp.of(60);
  public static final Current BAG_CURRENT_LIMIT = Units.Amp.of(70);
  public static final double HEAD_ON_ANGLE_ERROR = 5;
}

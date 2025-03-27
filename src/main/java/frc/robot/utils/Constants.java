package frc.robot.utils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Constants {
  public static final Current KRAKEN_CURRENT_LIMIT = Units.Amp.of(70);
  public static final Current KRAKEN_CURRENT_LOWER_LIMIT = Units.Amp.of(40);
  public static final Current NEO_550_CURRENT_LIMIT = Units.Amp.of(25);
  public static final Current NEO_CURRENT_LIMIT = Units.Amp.of(60);
  public static final Current BAG_CURRENT_LIMIT = Units.Amp.of(70);
  public static final AngularVelocity SPEAKER_SPEED = Units.RPM.of(3100);
  public static final Angle SPEAKER_ANGLE = Units.Rotations.of(0.852);

  public static final LoggedNetworkBoolean tuningMode = new LoggedNetworkBoolean("TuningMode", false);
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final Mode MODE;
  private static final boolean enableReplay = false;

  static {
    if (RobotBase.isReal()) MODE = Mode.REAL;
    else if (enableReplay) MODE = Mode.REPLAY;
    else MODE = Mode.SIM;
  }
}

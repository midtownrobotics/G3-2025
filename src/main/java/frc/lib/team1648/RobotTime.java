package frc.lib.team1648;

import org.littletonrobotics.junction.Logger;

public class RobotTime {
  /**
   * Returns non-deterministic timestamp
   *
   * @return Number of seconds on timer
   */
  public static double getTimestampSeconds() {
    long micros = Logger.getTimestamp();
    return (double) micros * 1.0E-6;
  }
}

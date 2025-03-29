package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.dashboard.LoggedTunableNumber;

public class ElevatorConstants {
  // TODO: Find out max and min heights
  public static final Distance elevatorMaxHeight = Inches.of(69.5);
  public static final Distance elevatorMinHeight = Feet.of(0);

  public static final double kGearing = 14;

  public static final Angle absoluteEncoderOffset = Radians.of(4.022);

  public static final LoggedTunableNumber maxA = new LoggedTunableNumber("Elevator/maxA", 58);
  public static final LoggedTunableNumber maxV = new LoggedTunableNumber("Elevator/maxV", 11);

  public final class PID_SCORE {
    public static final LoggedTunableNumber s = new LoggedTunableNumber("Elevator/PID_SCORE/s", 0.11);
    public static final LoggedTunableNumber g = new LoggedTunableNumber("Elevator/PID_SCORE/g", 0.6);
    public static final LoggedTunableNumber v = new LoggedTunableNumber("Elevator/PID_SCORE/v", 0.1);
    public static final LoggedTunableNumber a = new LoggedTunableNumber("Elevator/PID_SCORE/a", 0.0);
    public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_SCORE/p", 0.73);
    public static final LoggedTunableNumber i = new LoggedTunableNumber("Elevator/PID_SCORE/i", 0.0);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_SCORE/d", 0.0);
  }

  public final class PID_CLIMB {
    public static final LoggedTunableNumber s = new LoggedTunableNumber("Elevator/PID_CLIMB/s", 0.0);
    public static final LoggedTunableNumber g = new LoggedTunableNumber("Elevator/PID_CLIMB/g", 0.0);
    public static final LoggedTunableNumber v = new LoggedTunableNumber("Elevator/PID_CLIMB/v", 0.0);
    public static final LoggedTunableNumber a = new LoggedTunableNumber("Elevator/PID_CLIMB/a", 0.0);
    public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_CLIMB/p", 1.5);
    public static final LoggedTunableNumber i = new LoggedTunableNumber("Elevator/PID_CLIMB/i", 0.0);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_CLIMB/d", 0.0);
  }
}

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.lib.LoggedTunableNumber;

public class ElevatorConstants {
    // TODO: Find out max and min heights
    public static final Distance elevatorMaxHeight = Inches.of(0);
    public static final Distance elevatorMinHeight = Inches.of(0);

  public final class PID_SCORE {
    public static final LoggedTunableNumber s = new LoggedTunableNumber("Elevator/PID_SCORE/s", 0.0);
    public static final LoggedTunableNumber g = new LoggedTunableNumber("Elevator/PID_SCORE/g", 0.0);
    public static final LoggedTunableNumber v = new LoggedTunableNumber("Elevator/PID_SCORE/v", 0.0);
    public static final LoggedTunableNumber a = new LoggedTunableNumber("Elevator/PID_SCORE/a", 0.0);
    public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_SCORE/p", 0.0);
    public static final LoggedTunableNumber i = new LoggedTunableNumber("Elevator/PID_SCORE/i", 0.0);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_SCORE/d", 0.0);
  }

  public final class PID_CLIMB {
    public static final LoggedTunableNumber s = new LoggedTunableNumber("Elevator/PID_CLIMB/s", 0.0);
    public static final LoggedTunableNumber g = new LoggedTunableNumber("Elevator/PID_CLIMB/g", 0.0);
    public static final LoggedTunableNumber v = new LoggedTunableNumber("Elevator/PID_CLIMB/v", 0.0);
    public static final LoggedTunableNumber a = new LoggedTunableNumber("Elevator/PID_CLIMB/a", 0.0);
    public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_CLIMB/p", 0.0);
    public static final LoggedTunableNumber i = new LoggedTunableNumber("Elevator/PID_CLIMB/i", 0.0);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_CLIMB/d", 0.0);
  }
}

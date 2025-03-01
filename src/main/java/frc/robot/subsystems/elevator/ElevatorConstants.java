package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.dashboard.LoggedTunableNumber;

public class ElevatorConstants {
    // TODO: Find out max and min heights
    public static final Distance elevatorMaxHeight = Feet.of(5.5);
    public static final Distance elevatorMinHeight = Feet.of(0);

    public static final Angle absoluteEncoderOffset = Radians.of(5.143);

  public final class PID_SCORE {
    public static final LoggedTunableNumber s = new LoggedTunableNumber("Elevator/PID_SCORE/s", 0.12);
    public static final LoggedTunableNumber g = new LoggedTunableNumber("Elevator/PID_SCORE/g", 0.29);
    public static final LoggedTunableNumber v = new LoggedTunableNumber("Elevator/PID_SCORE/v", 0.08);
    public static final LoggedTunableNumber a = new LoggedTunableNumber("Elevator/PID_SCORE/a", 0.0);
    public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_SCORE/p", 0.55);
    public static final LoggedTunableNumber i = new LoggedTunableNumber("Elevator/PID_SCORE/i", 0.06);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_SCORE/d", 0.0);
  }

  public final class PID_CLIMB {
    public static final LoggedTunableNumber s = new LoggedTunableNumber("Elevator/PID_CLIMB/s", 0.08);
    public static final LoggedTunableNumber g = new LoggedTunableNumber("Elevator/PID_CLIMB/g", 0.0);
    public static final LoggedTunableNumber v = new LoggedTunableNumber("Elevator/PID_CLIMB/v", 0.0);
    public static final LoggedTunableNumber a = new LoggedTunableNumber("Elevator/PID_CLIMB/a", 0.0);
    public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_CLIMB/p", 0.55);
    public static final LoggedTunableNumber i = new LoggedTunableNumber("Elevator/PID_CLIMB/i", 0.0);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_CLIMB/d", 0.0);
  }
}

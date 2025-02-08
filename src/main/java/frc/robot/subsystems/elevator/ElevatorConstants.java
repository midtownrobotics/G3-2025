package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.lib.LoggedTunableNumber;

public class ElevatorConstants {
  // TODO: Find out max and min heights
  public static final Distance elevatorMaxHeight = Inches.of(0);
  public static final Distance elevatorMinHeight = Inches.of(0);

  public final class PID_SCORE {
    public static final LoggedTunableNumber ks = new LoggedTunableNumber("Elevator/PID_ELEVATE/ks", 0.0);
    public static final LoggedTunableNumber kg = new LoggedTunableNumber("Elevator/PID_ELEVATE/kg", 0.0);
    public static final LoggedTunableNumber kv = new LoggedTunableNumber("Elevator/PID_ELEVATE/kv", 0.0);
    public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_ELEVATE/p", 0.0);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_ELEVATE/d", 0.0);
  
    public static final LoggedTunableNumber maxV = new LoggedTunableNumber("Elevator/PID_ELEVATE/maxV", 0.0);
    public static final LoggedTunableNumber maxA = new LoggedTunableNumber("Elevator/PID_ELEVATE/maxA", 0.0);
  }

  public final class PID_CLIMB {
    public static final LoggedTunableNumber ks = new LoggedTunableNumber("Elevator/PID_CLIMB/ks", 0.0);
    public static final LoggedTunableNumber kg = new LoggedTunableNumber("Elevator/PID_CLIMB/kg", 0.0);
    public static final LoggedTunableNumber kv = new LoggedTunableNumber("Elevator/PID_CLIMB/kv", 0.0);
    public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_CLIMB/p", 0.0);
    public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_CLIMB/d", 0.0);
  
    public static final LoggedTunableNumber maxV = new LoggedTunableNumber("Elevator/PID_CLIMB/maxV", 0.0);
    public static final LoggedTunableNumber maxA = new LoggedTunableNumber("Elevator/PID_CLIMB/maxA", 0.0);
  }
}

package frc.robot.subsystems.algae_claw;

import frc.lib.LoggedTunableNumber;

public class AlgaeClawConstants {
    public final class PID_SCORE {
        public static final LoggedTunableNumber ks = new LoggedTunableNumber("Elevator/PID_ELEVATE/ks", 0.0);
        public static final LoggedTunableNumber kg = new LoggedTunableNumber("Elevator/PID_ELEVATE/kg", 0.0);
        public static final LoggedTunableNumber kv = new LoggedTunableNumber("Elevator/PID_ELEVATE/kv", 0.0);
        public static final LoggedTunableNumber p = new LoggedTunableNumber("Elevator/PID_ELEVATE/p", 0.0);
        public static final LoggedTunableNumber d = new LoggedTunableNumber("Elevator/PID_ELEVATE/d", 0.0);
    }
}

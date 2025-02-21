package frc.lib.dashboard;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class LTArmFeedForward {
  /** kS */
  private final LoggedTunableNumber m_kS;

  /** kG */
  private final LoggedTunableNumber m_kG;

  /** kV */
  private final LoggedTunableNumber m_kV;

  /** kA */
  private final LoggedTunableNumber m_kA;

  private ArmFeedforward m_feedforward;

  /** Empty contstructor makes all 0 */
  public LTArmFeedForward(String path) {
    this(path, 0, 0, 0, 0);
  }
  /** Pid controller has loggedtunablenumbers */
  public LTArmFeedForward(
      String path, double defaultKs, double defaultKg, double defaultKv, double defaultKa) {
    m_kS = new LoggedTunableNumber(path + "/kS", defaultKs);
    m_kG = new LoggedTunableNumber(path + "/kG", defaultKg);
    m_kV = new LoggedTunableNumber(path + "/kV", defaultKv);
    m_kA = new LoggedTunableNumber(path + "/kA", defaultKa);

    m_feedforward = new ArmFeedforward(defaultKs, defaultKg, defaultKv, defaultKa);
  }
  /** Updates pid constants if changed */
  public void updateValues() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_feedforward = new ArmFeedforward(m_kS.get(), m_kG.get(), m_kV.get(), m_kA.get());
        },
        m_kS,
        m_kG,
        m_kV,
        m_kA);
  }

  /** return feedforward for actual use */
  public ArmFeedforward getController() {
    return m_feedforward;
  }

  public double calculate(Angle positionSetpoint, AngularVelocity velocitySetpoint) {
    return m_feedforward.calculate(positionSetpoint.in(Radians), velocitySetpoint.in(RadiansPerSecond));
  }
}

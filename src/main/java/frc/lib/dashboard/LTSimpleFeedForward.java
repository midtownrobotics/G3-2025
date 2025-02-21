package frc.lib.dashboard;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;

public class LTSimpleFeedForward {
  /** kS */
  private final LoggedTunableNumber m_kS;

  /** kV */
  private final LoggedTunableNumber m_kV;

  /** kA */
  private final LoggedTunableNumber m_kA;

  private SimpleMotorFeedforward m_feedforward;

  /** Empty contstructor makes all 0 */
  public LTSimpleFeedForward(String path) {
    this(path, 0, 0, 0, 0);
  }
  /** Pid controller has loggedtunablenumbers */
  public LTSimpleFeedForward(
      String path, double defaultKs, double defaultKg, double defaultKv, double defaultKa) {
    m_kS = new LoggedTunableNumber(path + "/kS", defaultKs);
    m_kV = new LoggedTunableNumber(path + "/kV", defaultKv);
    m_kA = new LoggedTunableNumber(path + "/kA", defaultKa);

    m_feedforward = new SimpleMotorFeedforward(defaultKs, defaultKg, defaultKv, defaultKa);
  }
  /** Updates pid constants if changed */
  public void updateValues() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_feedforward = new SimpleMotorFeedforward(m_kS.get(), m_kV.get(), m_kA.get());
        },
        m_kS,
        m_kV,
        m_kA);
  }

  /** return feedforward for actual use */
  public SimpleMotorFeedforward getController() {
    return m_feedforward;
  }

  public double calculate(AngularVelocity velocitySetpoint) {
    return m_feedforward.calculate(velocitySetpoint.in(RadiansPerSecond));
  }
}

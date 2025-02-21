package frc.lib.dashboard;

import java.util.function.Consumer;

import com.pathplanner.lib.config.PIDConstants;

public class LoggedTunablePID {
      /** p */
  private final LoggedTunableNumber m_kP;
  /** i */
  private final LoggedTunableNumber m_kI;
  /** d */
  private final LoggedTunableNumber m_kD;

  public LoggedTunablePID(String path, double defaultP, double defaultI, double defaultD) {
    m_kP = new LoggedTunableNumber(path + "/1__kP", defaultP);
    m_kI = new LoggedTunableNumber(path + "/2__kI", defaultI);
    m_kD = new LoggedTunableNumber(path + "/3__kD", defaultD);
  }

  public void ifChanged(int hashCode, Consumer<PIDConstants> consumer) {
    LoggedTunableNumber.ifChanged(
        hashCode,
        () -> {
          consumer.accept(new PIDConstants(m_kP.get(), m_kI.get(), m_kD.get()));
        },
        m_kP,
        m_kI,
        m_kD);
  }
}

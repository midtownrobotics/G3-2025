package frc.lib.dashboard;

import com.pathplanner.lib.config.PIDConstants;
import java.util.function.Consumer;

public class LoggedTunablePID {
      /** p */
  private final LoggedTunableNumber m_kP;
  /** i */
  private final LoggedTunableNumber m_kI;
  /** d */
  private final LoggedTunableNumber m_kD;

  public LoggedTunablePID(String path, double defaultP, double defaultI, double defaultD) {
    m_kP = new LoggedTunableNumber(path + "/kP", defaultP);
    m_kI = new LoggedTunableNumber(path + "/kI", defaultI);
    m_kD = new LoggedTunableNumber(path + "/kD", defaultD);
  }

  public double getKp() {
    return m_kP.get();
  }

  public double getKi() {
    return m_kI.get();
  }

  public double getKd() {
    return m_kD.get();
  }

  public PIDConstants getConstants() {
    return new PIDConstants(m_kP.get(), m_kI.get(), m_kD.get());
  }

  public void ifChanged(int hashCode, Consumer<PIDConstants> consumer) {
    LoggedTunableNumber.ifChanged(
        hashCode,
        () -> {
          consumer.accept(getConstants());
        },
        m_kP,
        m_kI,
        m_kD);
  }
}

package frc.lib;

import edu.wpi.first.math.controller.PIDController;

public class LTPIDController {
  /** p */
  private final LoggedTunableNumber m_kP;
  /** i */
  private final LoggedTunableNumber m_kI;
  /** d */
  private final LoggedTunableNumber m_kD;
  /** dip controller */
  private final PIDController m_controller;

  /** Empty contstructor makes all 0 */
  public LTPIDController(String path) {
    this(path, 0, 0, 0);
  }
  /** Pid controller has loggedtunablenumbers */
  public LTPIDController(
      String path, double defaultP, double defaultI, double defaultD) {
    m_kP = new LoggedTunableNumber(path + "/P", defaultP);
    m_kI = new LoggedTunableNumber(path + "/I", defaultI);
    m_kD = new LoggedTunableNumber(path + "/D", defaultD);
    m_controller = new PIDController(defaultP, defaultI, defaultD);
  }
  /** Updates pid constants if changed */
  public void updateValues() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_controller.setPID(m_kP.get(), m_kI.get(), m_kD.get());
        },
        m_kD,
        m_kI,
        m_kP);
  }
  /** return pid controller for actual use */
  public PIDController getController() {
    return m_controller;
  }
}

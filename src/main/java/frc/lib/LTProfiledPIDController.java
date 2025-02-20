package frc.lib;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class LTProfiledPIDController {
  /** p */
  private final LoggedTunableNumber m_kP;
  /** i */
  private final LoggedTunableNumber m_kI;
  /** d */
  private final LoggedTunableNumber m_kD;

  /** maxV */
  private final LoggedTunableNumber m_maxV;

  private final LoggedTunableNumber m_maxA;

  /** dip controller */
  private final ProfiledPIDController m_controller;

  /** Empty contstructor makes all 0 */
  public LTProfiledPIDController(String path) {
    this(path, 0, 0, 0, 0, 0);
  }

  /** Pid controller has loggedtunablenumbers */
  public LTProfiledPIDController(
      String path,
      double defaultP,
      double defaultI,
      double defaultD,
      double defaultMaxVel,
      double defualtMaxAcc) {
    m_kP = new LoggedTunableNumber(path + "/P", defaultP);
    m_kI = new LoggedTunableNumber(path + "/I", defaultI);
    m_kD = new LoggedTunableNumber(path + "/D", defaultD);
    m_maxV = new LoggedTunableNumber(path + "/MaxV", defaultMaxVel);
    m_maxA = new LoggedTunableNumber(path + "/MaxA", defualtMaxAcc);
    m_controller =
        new ProfiledPIDController(
            defaultP, defaultI, defaultD, new Constraints(defaultMaxVel, defualtMaxAcc));
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
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_controller.setConstraints(new Constraints(m_maxV.get(), m_maxA.get()));
        },
        m_maxV,
        m_maxA);
  }

  /** return pid controller for actual use */
  public ProfiledPIDController getController() {
    return m_controller;
  }
}

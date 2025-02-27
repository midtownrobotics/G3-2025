package frc.lib.dashboard;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class LTProfiledPIDController {

  private final LoggedTunablePID m_pid;

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
    m_pid = new LoggedTunablePID(path, defaultP, defaultI, defaultD);
    m_maxV = new LoggedTunableNumber(path + "/Constraints/maxVelocity", defaultMaxVel);
    m_maxA = new LoggedTunableNumber(path + "/Constraints/maxAcceleration", defualtMaxAcc);
    m_controller =
        new ProfiledPIDController(
            defaultP, defaultI, defaultD, new Constraints(defaultMaxVel, defualtMaxAcc));
  }

  /** Updates pid constants if changed */
  public void updateValues() {
    m_pid.ifChanged(
        hashCode(),
        (pid) -> {
          m_controller.setPID(pid.kP, pid.kI, pid.kD);
        });
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

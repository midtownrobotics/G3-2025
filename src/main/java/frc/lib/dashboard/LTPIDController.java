package frc.lib.dashboard;

import edu.wpi.first.math.controller.PIDController;

public class LTPIDController {

  private final LoggedTunablePID m_pid;
  private final PIDController m_controller;

  /** Empty contstructor makes all 0 */
  public LTPIDController(String path) {
    this(path, 0, 0, 0);
  }
  /** Pid controller has loggedtunablenumbers */
  public LTPIDController(
      String path, double defaultP, double defaultI, double defaultD) {
    m_pid = new LoggedTunablePID(path, defaultP, defaultI, defaultD);
    m_controller = new PIDController(defaultP, defaultI, defaultD);
  }
  /** Updates pid constants if changed */
  public void updateValues() {
    m_pid.ifChanged(
        hashCode(),
        (pid) -> {
          m_controller.setPID(pid.kP, pid.kI, pid.kD);
        });
  }

  /** return pid controller for actual use */
  public PIDController getController() {
    return m_controller;
  }
}

package frc.lib.dashboard;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.dashboard.LoggedTunableMeasures.LoggedTunableLinearAcceleration;
import frc.lib.dashboard.LoggedTunableMeasures.LoggedTunableLinearVelocity;

public class LTLinearProfiledPIDController {

  private final LoggedTunablePID m_pid;

  private final LoggedTunableLinearVelocity m_maxV;

  private final LoggedTunableLinearAcceleration m_maxA;

  /** dip controller */
  private final ProfiledPIDController m_controller;

  /** Empty contstructor makes all 0 */
  public LTLinearProfiledPIDController(String path) {
    this(path, 0, 0, 0, MetersPerSecond.zero(), MetersPerSecondPerSecond.zero());
  }

  /** Pid controller has loggedtunablenumbers */
  public LTLinearProfiledPIDController(
      String path,
      double defaultP,
      double defaultI,
      double defaultD,
      LinearVelocity defaultMaxVel,
      LinearAcceleration defualtMaxAcc) {
    m_pid = new LoggedTunablePID(path, defaultP, defaultI, defaultD);
    m_maxV = new LoggedTunableLinearVelocity(path + "/Constraints/maxVelocity", defaultMaxVel);
    m_maxA = new LoggedTunableLinearAcceleration(path + "/Constraints/maxAcceleration", defualtMaxAcc);
    m_controller =
        new ProfiledPIDController(
            defaultP, defaultI, defaultD, new Constraints(defaultMaxVel.in(MetersPerSecond), defualtMaxAcc.in(MetersPerSecondPerSecond)));
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
          m_controller.setConstraints(new Constraints(m_maxV.get().in(MetersPerSecond), m_maxA.get().in(MetersPerSecondPerSecond)));
        },
        m_maxV.getTunableNumber(),
        m_maxA.getTunableNumber());
  }

  /** return pid controller for actual use */
  public ProfiledPIDController getController() {
    return m_controller;
  }

  public double calculate(Distance measurement, Distance setpoint) {
    return m_controller.calculate(measurement.in(Meters), setpoint.in(Meters));
  }

  public double calculate(double measurement, double setpoint) {
    return m_controller.calculate(measurement, setpoint);
  }

  public State getSetpoint() {
    return m_controller.getSetpoint();
  }

  public Distance getTolerance() {
    return Meters.of(m_controller.getPositionTolerance());
  }

  public boolean atGoal() {
    return m_controller.atGoal();
  }

  public Distance getPositionError() {
    return Meters.of(m_controller.getPositionError());
  }

  public LinearVelocity getVelocityError() {
    return MetersPerSecond.of(m_controller.getVelocityError());
  }
}

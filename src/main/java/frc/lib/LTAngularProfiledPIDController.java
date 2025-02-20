package frc.lib;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.LoggedTunableMeasures.LoggedTunableAngularAcceleration;
import frc.lib.LoggedTunableMeasures.LoggedTunableAngularVelocity;

public class LTAngularProfiledPIDController {
  /** p */
  private final LoggedTunableNumber m_kP;
  /** i */
  private final LoggedTunableNumber m_kI;
  /** d */
  private final LoggedTunableNumber m_kD;

  /** maxV */
  private final LoggedTunableAngularVelocity m_maxV;

  private final LoggedTunableAngularAcceleration m_maxA;

  /** dip controller */
  private final ProfiledPIDController m_controller;

  /** Empty contstructor makes all 0 */
  public LTAngularProfiledPIDController(String path) {
    this(path, 0, 0, 0, RadiansPerSecond.zero(), RadiansPerSecondPerSecond.zero());
  }

  /** Pid controller has loggedtunablenumbers */
  public LTAngularProfiledPIDController(
      String path,
      double defaultP,
      double defaultI,
      double defaultD,
      AngularVelocity defaultMaxVel,
      AngularAcceleration defualtMaxAcc) {
    m_kP = new LoggedTunableNumber(path + "/P", defaultP);
    m_kI = new LoggedTunableNumber(path + "/I", defaultI);
    m_kD = new LoggedTunableNumber(path + "/D", defaultD);
    m_maxV = new LoggedTunableAngularVelocity(path + "/MaxV", defaultMaxVel);
    m_maxA = new LoggedTunableAngularAcceleration(path + "/MaxA", defualtMaxAcc);
    m_controller =
        new ProfiledPIDController(
            defaultP, defaultI, defaultD, new Constraints(defaultMaxVel.in(RadiansPerSecond), defualtMaxAcc.in(RadiansPerSecondPerSecond)));
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
          m_controller.setConstraints(new Constraints(m_maxV.get().in(RadiansPerSecond), m_maxA.get().in(RadiansPerSecondPerSecond)));
        },
        m_maxV.getTunableNumber(),
        m_maxA.getTunableNumber());
  }

  /** return pid controller for actual use */
  public ProfiledPIDController getController() {
    return m_controller;
  }

    public double calculate(Angle measurement, Angle setpoint) {
    return m_controller.calculate(measurement.in(Radians), setpoint.in(Radians));
  }

  public double calculate(double measurement, double setpoint) {
    return m_controller.calculate(measurement, setpoint);
  }

  public State getSetpoint() {
    return m_controller.getSetpoint();
  }

  public Angle getTolerance() {
    return Radians.of(m_controller.getPositionTolerance());
  }

  public boolean atGoal() {
    return m_controller.atGoal();
  }

  public Angle getPositionError() {
    return Radians.of(m_controller.getPositionError());
  }

  public AngularVelocity getVelocityError() {
    return RadiansPerSecond.of(m_controller.getVelocityError());
  }
}

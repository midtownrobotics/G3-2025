package frc.robot.subsystems.elevator.winch;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class WinchIOSim implements WinchIO {
  private Distance height = Feet.zero();
  private final ElevatorSim m_sim;
  private final ElevatorFeedforward m_feedforward;
  private final ProfiledPIDController m_controller;

  /**
   * Creates a new WinchIOSim.
   */
  public WinchIOSim() {
    m_feedforward = new ElevatorFeedforward(0.08, 0.9, 0.2);
    m_controller = new ProfiledPIDController(20.0, 2.0, 0, new Constraints(Units.feetToMeters(8), Units.feetToMeters(8)));
    m_sim = new ElevatorSim(DCMotor.getKrakenX60(2), 20, 3, 1.075 / 2, 0, Units.feetToMeters(5.5), true, 0, 0.001, 0.001);
  }

  @Override
  public void updateInputs(WinchInputs inputs) {
    // double pidVoltage = m_controller.calculate(m_sim.getPositionMeters(), height.in(Meters));
    // double ffVoltage = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    // double voltage = pidVoltage + ffVoltage;
    // m_sim.setInputVoltage(voltage);
    // m_sim.update(0.02);

    inputs.left.position = height; // Meters.of(m_sim.getPositionMeters());
    inputs.right.position = height; // Meters.of(m_sim.getPositionMeters());
    inputs.left.velocity = MetersPerSecond.of(m_sim.getVelocityMetersPerSecond());
    inputs.right.velocity = MetersPerSecond.of(m_sim.getVelocityMetersPerSecond());
  }

  @Override
  public void setScorePosition(Distance position) {
    height = position;
  }

  @Override
  public void setClimbPosition(Distance position) {
    height = position;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public void zeroPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'zeroPosition'");
  }

  @Override
  public void zeroPosition(Angle position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'zeroPosition'");
  }
}

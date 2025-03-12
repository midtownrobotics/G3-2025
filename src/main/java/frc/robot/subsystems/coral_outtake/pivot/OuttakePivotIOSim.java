package frc.robot.subsystems.coral_outtake.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class OuttakePivotIOSim implements OuttakePivotIO {
  private final SingleJointedArmSim m_sim;
  private final MutVoltage voltage = Volts.mutable(0);

  /**
   * Creates a new PivotIOSim.
   */
  public OuttakePivotIOSim() {
    m_sim = new SingleJointedArmSim(DCMotor.getNEO(1), 20, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), 1.5), Units.inchesToMeters(10), Units.degreesToRadians(-10), Units.degreesToRadians(141), true, 0, 0.001, 0.001);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    m_sim.setInputVoltage(voltage.in(Volts));
    m_sim.update(0.02);

    inputs.position = Radians.of(m_sim.getAngleRads());
    inputs.absolutePosition = Radians.of(m_sim.getAngleRads());
    inputs.velocity = RadiansPerSecond.of(m_sim.getVelocityRadPerSec());
    inputs.supplyCurrent = Amps.of(m_sim.getCurrentDrawAmps());
    inputs.appliedVoltage = voltage;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    this.voltage.mut_replace(voltage);
  }

  @Override
  public void setPosition(Angle angle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }
}

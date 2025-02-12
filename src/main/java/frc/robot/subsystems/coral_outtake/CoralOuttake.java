package frc.robot.subsystems.coral_outtake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralOuttake extends SubsystemBase {

  public enum State {
    // TODO: Set correct voltages
    IDLE(0),
    SHOOT(12),
    REVERSE_HANDOFF(-12),
    HANDOFF(12),
    STATION_INTAKE(12),
    TUNING(),
    MANUAL();

    private @Getter Voltage voltage;

    /** State has no meter height value associated */
    private State() {
      this.voltage = null;
    }

    State(double voltage) {
      this.voltage = Volts.of(voltage);
    }
  }

  private @Getter State currentState = State.IDLE;
  private RollerIO rollerIO;
  private RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

  /**
   * Initializes Coral Outtake
   * @param rollerIO
   */
  public CoralOuttake(RollerIO rollerIO) {
    this.rollerIO = rollerIO;
  }

  public AngularVelocity getRollerSpeed() {
    return rollerInputs.velocity;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);

    // state switch case
    switch (getCurrentState()) {
      case TUNING:
      case MANUAL:
        rollerIO.setVoltage(Units.Volts.zero());
        break;
      default:
        rollerIO.setVoltage(getCurrentState().getVoltage());
        break;
    }

    Logger.recordOutput("CoralOuttake/currentState", getCurrentState());
    Logger.recordOutput("CoralOuttake/desiredVoltage", getCurrentState().getVoltage());

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(State state) {
    currentState = state;
  }

  public Voltage getRollerVoltage() {
    return rollerInputs.appliedVoltage;
  }
}

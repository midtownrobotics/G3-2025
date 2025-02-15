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
import lombok.RequiredArgsConstructor;

import org.littletonrobotics.junction.Logger;


@RequiredArgsConstructor
public class CoralOuttake extends SubsystemBase {

  private final RollerIO rollerIO;

  public enum Goal {
    // TODO: Set correct voltages
    IDLE(Volts.zero()),
    SHOOT(CoralOuttakeConstants.SHOOT_VOLTAGE),
    REVERSE_HANDOFF(CoralOuttakeConstants.HANDOFF_VOLTAGE.times(-1)),
    HANDOFF(CoralOuttakeConstants.HANDOFF_VOLTAGE),
    STATION_INTAKE(CoralOuttakeConstants.STATION_INTAKE_VOLTAGE),
    TUNING(Volts.zero()),
    MANUAL(Volts.zero());

    private @Getter Voltage voltage;

    Goal(Voltage voltage) {
      this.voltage = voltage;
    }
  }

  private @Getter Goal currentGoal = Goal.IDLE;
  private RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

  public AngularVelocity getRollerSpeed() {
    return rollerInputs.velocity;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);

    // goal switch case
    switch (getCurrentGoal()) {
      case TUNING:
      case MANUAL:
        rollerIO.setVoltage(Units.Volts.zero());
        break;
      default:
        rollerIO.setVoltage(getCurrentGoal().getVoltage());
        break;
    }

    Logger.recordOutput("CoralOuttake/currentState", getCurrentGoal());
    Logger.recordOutput("CoralOuttake/desiredVoltage", getCurrentGoal().getVoltage());

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(Goal goal) {
    currentGoal = goal;
  }

  public Voltage getRollerVoltage() {
    return rollerInputs.appliedVoltage;
  }
}

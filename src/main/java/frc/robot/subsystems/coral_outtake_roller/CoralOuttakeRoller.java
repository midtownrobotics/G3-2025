package frc.robot.subsystems.coral_outtake_roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakeRoller extends SubsystemBase {

  public enum Goal {
    STOW(Volts.zero()),
    SHOOT(Volts.of(5.5)),
    DEALGIFY(Volts.of(5.5)),
    HANDOFF(Volts.of(3)),
    HANDOFF_REVERSE(Volts.of(-2)),
    INTAKE(Volts.zero()),
    REVERSE_SHOOT(Volts.of(-7)),
    TUNING(),
    MANUAL();

    private @Getter Voltage volts;

    private Goal(Voltage voltage) {
      this.volts = voltage;
    }

    private Goal() {

    }
  }

  public final Trigger currentSpikeTrigger;

  private @Getter Goal currentRollerGoal = Goal.STOW;

  private RollerIO rollerIO;

  private RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

  /**
   * Initializes Coral Outtake
   *
   * @param rollerIO
   */
  public CoralOuttakeRoller(RollerIO rollerIO) {
    this.rollerIO = rollerIO;

    currentSpikeTrigger = new Trigger(this::currentSpikeFiltered);
  }

  private LinearFilter currentSpikeFilter = LinearFilter.movingAverage(3);

  private boolean currentSpikeFiltered() {
    return currentSpikeFilter.calculate(rollerInputs.supplyCurrent.in(Amps)) > 10;
  }

  public AngularVelocity getRollerSpeed() {
    return rollerInputs.velocity;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);

    rollerIO.setVoltage(getCurrentRollerGoal().getVolts());

    Logger.recordOutput("CoralOuttake/currentRollerGoal", getCurrentRollerGoal());

    Logger.recordOutput("CoralOuttake/desiredVoltage", getCurrentRollerGoal().getVolts());
    Logger.recordOutput("CoralOuttake/currentSpikeTrigger", currentSpikeTrigger);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(Goal goal) {
    currentRollerGoal = goal;
  }

  /**
   * Returns the voltage of the roller motor
   */
  public Voltage getRollerVoltage() {
    return rollerInputs.appliedVoltage;
  }

  /**
   * Returns a command that sets the goal of the coral outtake.
   */
  public Command setGoalCommand(Goal goal) {
    return runOnce(() -> setGoal(goal));
  }

  /** Returns a command that sets the goal of the coral outtake and resets the goal when it ends */
  public Command setGoalEndCommand(Goal goal, Goal endGoal) {
    return run(() -> setGoal(goal)).finallyDo(() -> setGoal(endGoal));
  }
}

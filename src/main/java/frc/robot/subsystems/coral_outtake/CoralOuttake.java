package frc.robot.subsystems.coral_outtake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.robot.sensors.Photoelectric;
import frc.robot.subsystems.coral_outtake.pivot.OuttakePivotIO;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;

public class CoralOuttake extends SubsystemBase {

  public enum Goal {
    // TODO: Set correct voltages
    STOW(Volts.zero(), Degrees.of(0)),
    CORAL_BACKWARDS(Volts.of(3), Degrees.of(0)),
    REVERSE_SHOOT(Volts.of(5), Degrees.of(0)),
    CORAL_TINY_ADJUST(Volts.of(-1), Degrees.of(0)),
    SHOOT(Volts.of(-10), Degrees.of(0)),
    HANDOFF(Volts.of(-7), Degrees.of(0)),
    SLOWER_HANDOFF(Volts.of(-5.5), Degrees.of(0)),
    TUNING(),
    MANUAL();

    private @Getter Voltage voltage;
    private @Getter Angle position;

    /** Goal has no meter height value associated */
    private Goal() {
      this.voltage = null;
    }

    Goal(Voltage voltage, Angle position) {
      this.voltage = voltage;
      this.position = position;
    }
  }

  public final Trigger currentSpikeTrigger;

  private @Getter Goal currentGoal = Goal.STOW;
  private RollerIO rollerIO;
  private OuttakePivotIO pivotIO;
  private RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final Photoelectric handoffSensor;


  /**
   * Initializes Coral Outtake
   * @param rollerIO
   */
  public CoralOuttake(RollerIO rollerIO, OuttakePivotIO pivotIO, Photoelectric handoffSensor) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
    this.handoffSensor = handoffSensor;

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

    // goal switch case
    switch (getCurrentGoal()) {
      case TUNING:
      case MANUAL:
        rollerIO.setVoltage(Units.Volts.zero());
        break;
      case STOW:
          rollerIO.setVoltage(Volts.zero());
        break;
      default:
        rollerIO.setVoltage(getCurrentGoal().getVoltage());
        break;
    }

    pivotIO.setPosition(getCurrentGoal().getPosition());

    Logger.recordOutput("CoralOuttake/currentState", getCurrentGoal());
    Logger.recordOutput("CoralOuttake/desiredVoltage", getCurrentGoal().getVoltage());
    Logger.recordOutput("CoralOuttake/currentSpikeTrigger", currentSpikeTrigger);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(Goal goal) {
    currentGoal = goal;
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

  /**
   * Returns a command that sets the goal of the elevator and sets the goal to the endGoal when the command ends.
   */
  public Command setGoalEndCommand(Goal goal, Goal endGoal) {
    return run(() -> setGoal(goal)).finallyDo(() -> setGoal(endGoal));
  }
}

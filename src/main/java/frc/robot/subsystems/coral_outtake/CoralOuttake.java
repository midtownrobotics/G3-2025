package frc.robot.subsystems.coral_outtake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.robot.sensors.Photoelectric;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralOuttake extends SubsystemBase {

  public enum Goal {
    // TODO: Set correct voltages
    IDLE(0),
    CORAL_BACKWARDS(3),
    CORAL_TINY_ADJUST(-1),
    SHOOT(-12),
    HANDOFF(-4.5),
    TUNING(),
    MANUAL();

    private @Getter Voltage voltage;

    /** Goal has no meter height value associated */
    private Goal() {
      this.voltage = null;
    }

    Goal(double voltage) {
      this.voltage = Volts.of(voltage);
    }
  }

  public final Trigger currentSpikeTrigger;

  private @Getter Goal currentGoal = Goal.IDLE;
  private RollerIO rollerIO;
  private RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final Photoelectric handoffSensor;
  

  /**
   * Initializes Coral Outtake
   * @param rollerIO
   */
  public CoralOuttake(RollerIO rollerIO, Photoelectric handoffSensor) {
    this.rollerIO = rollerIO;
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
      case IDLE:
          rollerIO.setVoltage(Volts.zero());
        break;
      // case HANDOFF_PUSH_UP:
      //   if (handoffSensor.isTriggered()) {
      //     rollerIO.setVoltage(Volts.of(-4.2));
      //   } else {
      //     rollerIO.setVoltage(Volts.zero());
      //   }
      //   break;
      default:
        rollerIO.setVoltage(getCurrentGoal().getVoltage());
        break;
    }

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

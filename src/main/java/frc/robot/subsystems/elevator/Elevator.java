package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.elevator.lock.LockIO;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggerUtil;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public Distance driverOffset = Inches.zero();

  private LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(
      ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);

  public enum Goal {
    STOW(Feet.zero()),
    HANDOFF(Inches.of(0.5)),
    L2(Inches.of(24)),
    L3(Inches.of(40)),
    L4(Inches.of(65.25)),
    PROCESSOR(Inches.zero()),
    DEALGIFY_LOW(Inches.of(13.5)),
    DEALGIFY_HIGH(Inches.of(30)),
    BARGE(Inches.of(68)),
    CLIMB(Inches.of(16)),
    CLIMB_BOTTOM(Feet.zero(), false),
    CLIMB_BOTTOM_LOCK(CLIMB_BOTTOM.getHeight(), true),
    TUNING(Feet.zero()),
    ZERO(Feet.zero()),
    MANUAL(Feet.zero());

    private @Getter Distance height;
    private @Getter boolean lockEnabled;

    /** Goal has no meter height value associated */
    private Goal() {
      this.height = null;
      lockEnabled = false;
    }

    /** Goal has meter height value associated */
    private Goal(Distance height) {
      this.height = height;
      lockEnabled = false;
    }

    private Goal(Distance height, boolean lockEnabled) {
      this.height = height;
      this.lockEnabled = lockEnabled;
    }

    /**
     * Converts a CoralMode to an Elevator Goal
     */
    public static Goal fromCoralMode(CoralMode mode) {
      return switch (mode) {
        case L2 -> L2;
        case L3 -> L3;
        case L4 -> L4;
        default -> STOW;
      };
    }

    /**
     * Converts a CoralMode to an Elevator dealgify Goal
     *
     * @param mode
     * @return
     */
    public static Goal dealgifyFromCoralMode(CoralMode mode) {
      return switch (mode) {
        case L1 -> Goal.DEALGIFY_LOW;
        case L2 -> Goal.DEALGIFY_LOW;
        case L3 -> Goal.DEALGIFY_HIGH;
        case L4 -> Goal.DEALGIFY_HIGH;
        default -> Goal.DEALGIFY_HIGH;
      };
    }
  }

  public final Trigger atGoalTrigger = new Trigger(this::atGoal);

  private @Getter Goal currentGoal = Goal.STOW;

  private WinchInputsAutoLogged winchInputs = new WinchInputsAutoLogged();
  private @Getter WinchIO winch;
  private @Getter LockIO lock;

  private SysIdRoutine routine;

  private LoggedTunableNumber tuningDesiredHeight = new LoggedTunableNumber("Elevator/Tuning/DesiredHeight", 0.0);

  /**
   * Constructs elevator :)
   *
   * @param winch
   */
  public Elevator(WinchIO winch, LockIO lock) {
    this.winch = winch;
    this.lock = lock;

    SysIdRoutine.Mechanism sysIdMech = new SysIdRoutine.Mechanism(
        winch::setVoltage,
        this::motorSysIdLog,
        this);

    routine = new SysIdRoutine(new Config(Volts.of(1).per(Second), Volts.of(1), Seconds.of(5)), sysIdMech);
  }

  private void motorSysIdLog(SysIdRoutineLog log) {
    log.motor("leftMotor")
        .voltage(winchInputs.left.appliedVoltage)
        .linearPosition(winchInputs.left.position)
        .linearVelocity(winchInputs.left.velocity);
    log.motor("rightMotor")
        .voltage(winchInputs.right.appliedVoltage)
        .linearPosition(winchInputs.right.position)
        .linearVelocity(winchInputs.right.velocity);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    winch.updateInputs(winchInputs);
    Logger.processInputs("Elevator", winchInputs);

    Distance constrainedHeight = elevatorConstraint.getClampedValue(getCurrentGoal().getHeight());

    if (getCurrentGoal() == Goal.L4 || getCurrentGoal() == Goal.L3 || getCurrentGoal() == Goal.L2
        || getCurrentGoal() == Goal.DEALGIFY_HIGH || getCurrentGoal() == Goal.DEALGIFY_LOW
        || getCurrentGoal() == Goal.BARGE || getCurrentGoal() == Goal.PROCESSOR) {
      constrainedHeight = elevatorConstraint.getClampedValue(getCurrentGoal().getHeight()).plus(driverOffset);
    }

    Distance desiredTuningHeight = Inches.of(tuningDesiredHeight.get());

    if (Constants.tuningMode.get()) {
      constrainedHeight = ((elevatorConstraint.getClampedValue(desiredTuningHeight)));
    }

    lock.setLockEnabled(getCurrentGoal().lockEnabled);

    switch (getCurrentGoal()) {
      case ZERO:
        winch.setVoltage(Volts.of(-1));
        break;
      case CLIMB_BOTTOM:
      case CLIMB_BOTTOM_LOCK:
        winch.setClimbPosition(constrainedHeight);
        break;
      case TUNING:
        break;
      case MANUAL:
      default:
        winch.setScorePosition(constrainedHeight);
        break;
    }

    Logger.recordOutput("Elevator/currentGoal", getCurrentGoal());
    Logger.recordOutput("Elevator/goalHeight", currentGoal.getHeight());
    Logger.recordOutput("Elevator/atGoal", atGoal());

    Logger.recordOutput("Elevator/currentHeight", getPosition());
    Logger.recordOutput("Elevator/currentVelocity", getVelocity());

    Logger.recordOutput("Elevator/constrainedMaxHeight", elevatorConstraint.getUpper());
    Logger.recordOutput("Elevator/constrainedMinHeight", elevatorConstraint.getLower());
    Logger.recordOutput("Elevator/constrainedGoalHeight", constrainedHeight);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the elevator. */
  public void setGoal(Goal goal) {
    currentGoal = goal;
  }

  public void setConstraints(LinearConstraint<DistanceUnit, Distance> constraints) {
    elevatorConstraint = constraints;
  }

  public Distance getPosition() {
    return winchInputs.left.position;
  }

  public LinearVelocity getVelocity() {
    return winchInputs.left.velocity;
  }

  /**
   * Runs the sysIdQuasistatic test on the elevator.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Runs the sysIdDynamic test on the elevator.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  /**
   * Returns true if the elevator is within a small threshold distance to the
   * goal.
   */
  public boolean atGoal() {
    return atGoal(getCurrentGoal());
  }

  /**
   * Returns true if the elevator is within a small threshold distance to the
   * goal.
   *
   * @param tolerance
   * @return
   */
  public boolean atGoal(Distance tolerance) {
    return atGoal(getCurrentGoal(), tolerance);
  }

  /**
   * Returns true if the elevator is within a small threshold distance to the
   * specified goal.
   */
  public boolean atGoal(Goal goal) {
    return atGoal(goal, Inches.of(0.99));
  }

  /**
   * Returns true if the elevator is within a small threshold distance to the
   * specified goal.
   */
  public boolean atGoal(Goal goal, Distance tolerance) {
    return getCurrentGoal() == goal && getPosition().isNear(goal.getHeight(), tolerance);
  }

  /**
   * Returns a trigger for if the elevator is within a small threshold distance to
   * the goal.
   */
  public Trigger atGoalTrigger(Goal goal) {
    return new Trigger(() -> atGoal(goal));
  }

  /**
   * Returns a command that sets the goal of the elevator and finishes
   * immediately.
   */
  public Command setGoalCommand(Goal goal) {
    return runOnce(() -> setGoal(goal));
  }

  /**
   * Returns a command that sets the supplied goal of the elevator and finishes
   * immediately.
   */
  public Command setGoalCommand(Supplier<Goal> goal) {
    return runOnce(() -> setGoal(goal.get()));
  }

  /**
   * Returns a command that sets the goal of the elevator and sets the goal to the
   * endGoal when the command ends.
   */
  public Command setGoalEndCommand(Goal goal, Goal endGoal) {
    return run(() -> setGoal(goal)).finallyDo(() -> setGoal(endGoal));
  }

  /**
   * Returns a command that sets the goal of the elevator and sets the goal to the
   * endGoal when the command ends.
   */
  public Command setGoalEndCommand(Supplier<Goal> goal, Goal endGoal) {
    return run(() -> setGoal(goal.get())).finallyDo(() -> setGoal(endGoal));
  }

  /**
   * Returns a command that sets the goal of the elevator and waits until the
   * elevator is at the goal.
   */
  public Command setGoalAndWait(Goal goal) {
    return run(() -> setGoal(goal)).until(this::atGoal);
  }

  /**
   * Returns a command that sets the goal of the elevator and waits until the
   * elevator is at the goal.
   */
  public Command setGoalAndWait(Goal goal, Distance tolerance) {
    return run(() -> setGoal(goal)).until(() -> atGoal(tolerance));
  }

  /**
   * Returns a command that sets the goal of the elevator using a supplier and
   * waits until the elevator is at the goal.
   */
  public Command setGoalAndWait(Supplier<Goal> goal) {
    return run(() -> setGoal(goal.get())).until(this::atGoal);
  }

  /**
   * Converts a CoralMode to an Elevator dealgify Goal.
   */
  public Command setDealgifyGoalFromCoralMode(Supplier<CoralMode> mode) {
    return Commands.run(() -> setGoal(Goal.dealgifyFromCoralMode(mode.get())), this);
  }
}

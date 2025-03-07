package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.utils.LoggerUtil;
import frc.robot.utils.UnitUtil;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);

  public enum Goal {
    STOW(Feet.zero()),
    HANDOFF(Inches.of(0.5)),
    L1(Inches.of(18)),
    L2(Inches.of(20)),
    L3(Inches.of(46.5)),
    AUTO_L4(Inches.of(62)),
    L4(Inches.of(60.5)),
    CLIMB(Feet.of(1.1)),
    CLIMB_BOTTOM(Feet.zero(), false),
    CLIMB_BOTTOM_LOCK(CLIMB_BOTTOM.getHeight(), true),
    TUNING(Feet.zero()),
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
        case L1 -> L1;
        case L2 -> L2;
        case L3 -> L3;
        case L4 -> L4;
        default -> STOW;
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
        this
    );

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

    Distance desiredTuningHeight = Feet.of(tuningDesiredHeight.get());

    desiredTuningHeight = UnitUtil.clamp(desiredTuningHeight, Feet.of(0), Feet.of(5.3));

    lock.setLockEnabled(getCurrentGoal().lockEnabled);

    switch (getCurrentGoal()) {
      case CLIMB_BOTTOM:
      case CLIMB_BOTTOM_LOCK:
        winch.setClimbPosition(constrainedHeight);
        break;
      case TUNING:
        winch.setClimbPosition(desiredTuningHeight);
        break;
      case MANUAL:
      default:
        winch.setScorePosition(constrainedHeight);
        break;
    }

    Logger.recordOutput("Elevator/currentGoal", getCurrentGoal());
    Logger.recordOutput("Elevator/goalHeightInches", currentGoal.getHeight().in(Inches));
    Logger.recordOutput("Elevator/atGoal", atGoal());

    Logger.recordOutput("Elevator/currentHeightInches", getPosition().in(Inches));
    Logger.recordOutput("Elevator/currentVelocityInchesPerSecond", getVelocity().in(InchesPerSecond));

    Logger.recordOutput("Elevator/constrainedMaxHeightInches", elevatorConstraint.getUpper().in(Inches));
    Logger.recordOutput("Elevator/constrainedMinHeightInches", elevatorConstraint.getLower().in(Inches));
    Logger.recordOutput("Elevator/constrainedGoalHeightInches", constrainedHeight.in(Inches));

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
   * Returns true if the elevator is within a small threshold distance to the goal.
   */
  public boolean atGoal() {
    return atGoal(getCurrentGoal());
  }

  /**
   * Returns true if the elevator is within a small threshold distance to the specified goal.
   */
  public boolean atGoal(Goal goal) {
    return getCurrentGoal() == goal && getPosition().isNear(goal.getHeight(), Inches.of(0.5));
  }

  /**
   * Returns a trigger for if the elevator is within a small threshold distance to the goal.
   */
  public Trigger atGoalTrigger(Goal goal) {
    return new Trigger(() -> atGoal(goal));
  }

  /**
   * Returns a command that sets the goal of the elevator and finishes immediately.
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

  /**
   * Returns a command that sets the goal of the elevator and sets the goal to the endGoal when the command ends.
   */
  public Command setGoalEndCommand(Supplier<Goal> goal, Goal endGoal) {
    return run(() -> setGoal(goal.get())).finallyDo(() -> setGoal(endGoal));
  }

  /**
   * Returns a command that sets the goal of the elevator and waits until the elevator is at the goal.
   */
  public Command setGoalAndWait(Goal goal) {
    return run(() -> setGoal(goal)).until(this::atGoal);
  }
}

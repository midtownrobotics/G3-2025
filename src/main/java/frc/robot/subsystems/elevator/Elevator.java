package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.LoggerUtil;
import frc.robot.utils.UnitUtil;
import lombok.Getter;

public class Elevator extends SubsystemBase {

  private LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);

  public enum Goal {
    STOW(Feet.zero()),
    L1(Feet.of(1.5)),
    L2(Feet.of((2.5))),
    L3(Feet.of(3.5)),
    L4(Feet.of(4.5)),
    CLIMB(Inches.of(5)),
    TUNING(Feet.zero()),
    MANUAL(Feet.zero());

    private @Getter Distance height;

    /** Goal has no meter height value associated */
    private Goal() {
      this.height = null;
    }

    /** Goal has meter height value associated */
    private Goal(Distance height) {
      this.height = height;
    }
  }

  public final Trigger atGoalTrigger = new Trigger(this::atGoal);

  private @Getter Goal currentGoal = Goal.STOW;

  private WinchInputsAutoLogged winchInputs = new WinchInputsAutoLogged();
  private @Getter WinchIO winch;

  private SysIdRoutine routine;

  private LoggedTunableNumber tuningDesiredHeight = new LoggedTunableNumber("Elevator/Tuning/DesiredHeight", 0.0);

  /**
   * Constructs elevator :)
   *
   * @param winch
   */
  public Elevator(WinchIO winch) {
    this.winch = winch;

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

    // winch.setClimbPosition(desiredTuningHeight);

    switch (getCurrentGoal()) {
      case CLIMB:
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

    Logger.recordOutput("Elevator/currentState", getCurrentGoal());
    Logger.recordOutput("Elevator/desiredHeight", currentGoal.getHeight().in(Inches));
    Logger.recordOutput("Elevator/constrainedMaxHeight", elevatorConstraint.getUpper().in(Inches));
    Logger.recordOutput("Elevator/constrainedMinHeight", elevatorConstraint.getLower().in(Inches));
    Logger.recordOutput("Elevator/constrainedHeight", constrainedHeight.in(Inches));
    Logger.recordOutput("Elevator/AtGoal", atGoal());
    Logger.recordOutput("Elevator/currentState", getCurrentGoal());

    Logger.recordOutput("Elevator/Tuning/DesiredHeightInches", desiredTuningHeight.in(Inches));
    Logger.recordOutput("Elevator/Tuning/CurrentHeightInches", getPosition().in(Inches));

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

  public Command setGoalCommand(Goal goal) {
    return runOnce(() -> setGoal(goal));
  }

  public Command setGoalEndCommand(Goal goal, Goal endGoal) {
    return run(() -> setGoal(goal)).finallyDo(() -> setGoal(endGoal));
  }

  public Command setGoalAndWait(Goal goal) {
    return run(() -> setGoal(goal)).until(this::atGoal);
  }

  public Command setGoalAndWait(Goal goal, Time timeout) {
    return setGoalAndWait(goal).withTimeout(timeout);
  }
}

package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;

public class Elevator extends SubsystemBase {

  private LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);

  public enum State {
    STOW(0),
    HANDOFF(0),
    L1(0),
    L2(0),
    L3(0),
    L4(0),
    BARGE(0),
    PROCESSOR(0),
    CLIMB(0),
    STATION(0),
    ALGAE_GROUND(0),
    ALGAE_STACKED(0),
    TUNING,
    MANUAL;

    private @Getter Distance height;

    /** State has no meter height value associated */
    private State() {
      this.height = null;
    }

    /** State has meter height value associated */
    private State(int height) {
      this.height = Units.Meters.of(height);
    }
  }

  private @Getter State currentState = State.STOW;

  private WinchInputsAutoLogged winchInputs = new WinchInputsAutoLogged();
  private @Getter WinchIO winch;

  private SysIdRoutine routine;

  /**
   * Constructs elevator :)
   *
   * @param winch
   */
  public Elevator(WinchIO winch) {
    this.winch = winch;
    winch.updateInputs(winchInputs);

      SysIdRoutine.Mechanism sysIdMech = new SysIdRoutine.Mechanism(
        winch::setVoltage,
        this::motorSysIdLog,
        this
    );

    routine = new SysIdRoutine(new Config(Volts.of(1).per(Second), Volts.of(1), Seconds.of(3)), sysIdMech);
  }

  private void motorSysIdLog(SysIdRoutineLog log) {
    log.motor("leftMotor")
      .voltage(winchInputs.left.appliedVoltage)
      .linearPosition(winchInputs.left.position)
      .linearVelocity(winchInputs.left.velocity);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    winch.updateInputs(winchInputs);

    switch (getCurrentState()) {
      case CLIMB:
        winch.setClimbPosition(elevatorConstraint.getClosestToDesired(getPosition(), currentState.height));
        break;
      case TUNING:
        break;
      default:
        winch.setScorePosition(elevatorConstraint.getClosestToDesired(getPosition(), currentState.height));
        break;
    }

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the elevator. */
  public void setGoal(State state, LinearConstraint<DistanceUnit, Distance> constraint) {
    currentState = state;
    elevatorConstraint = constraint;
  }

  public Distance getPosition() {
    return winch.getPosition();
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
}

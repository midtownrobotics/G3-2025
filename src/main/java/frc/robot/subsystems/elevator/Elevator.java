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
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);

  public enum Goal {
    STOW(0), // TODO
    HANDOFF(0), // TODO
    L1(0), // TODO
    L2(0), // TODO
    L3(0), // TODO
    L4(0), // TODO
    BARGE(0), // TODO
    PROCESSOR(0), // TODO
    CLIMB(0), // TODO
    STATION(0), // TODO
    ALGAE_GROUND(0), // TODO
    ALGAE_STACKED(0), // TODO
    TUNING(0), // TODO
    MANUAL(0); // TODO

    private @Getter Distance height;

    /** Goal has meter height value associated */
    private Goal(int height) {
      this.height = Units.Meters.of(height);
    }
  }

  private @Getter Goal currentGoal = Goal.STOW;

  private WinchInputsAutoLogged winchInputs = new WinchInputsAutoLogged();
  private WinchIO winch;

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
    log.motor("rightMotor")
      .voltage(winchInputs.right.appliedVoltage)
      .linearPosition(winchInputs.right.position)
      .linearVelocity(winchInputs.right.velocity);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    winch.updateInputs(winchInputs);

    switch (getCurrentGoal()) {
      case CLIMB:
        winch.setClimbPosition(elevatorConstraint.getClosestToDesired(getPosition(), currentGoal.height));
        break;
      case TUNING:
        break;
      default:
        winch.setScorePosition(elevatorConstraint.getClosestToDesired(getPosition(), currentGoal.height));
        break;
    }

    Logger.recordOutput("Elevator/currentState", getCurrentGoal());
    Logger.recordOutput("Elevator/desiredPosition", getCurrentGoal().getHeight());

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the elevator. */
  public void setGoal(Goal goal, LinearConstraint<DistanceUnit, Distance> constraint) {
    currentGoal = goal;
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

package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
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
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.LoggerUtil;
import frc.robot.utils.UnitUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);

  public enum Goal {
    STOW(0.05),
    HANDOFF(0.05),
    L1(0.2),
    L2(0.5),
    L3(0.75),
    L4(1.3),
    CLIMB(0.05),
    STATION(0.05),
    TUNING(0),
    MANUAL(0);

    private @Getter Distance height;

    /** Goal has no meter height value associated */
    private Goal() {
      this.height = null;
    }

    /** Goal has meter height value associated */
    private Goal(double height) {
      this.height = Units.Meters.of(height);
    }
  }

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

    Distance desiredTuningHeight = Feet.of(tuningDesiredHeight.get());

    desiredTuningHeight = UnitUtil.clamp(desiredTuningHeight, Feet.of(0), Feet.of(5.3));

    // winch.setClimbPosition(desiredTuningHeight);

    switch (getCurrentGoal()) {
      // case CLIMB:
      //   winch.setClimbPosition(elevatorConstraint.getClosestToDesired(getPosition(), currentGoal.height));
      //   break;
      // case TUNING:      //   winch.setClimbPosition(desiredTuningHeight);
      //   break;
      // case MANUAL:
      default:
        // Distance constrainedGoal = elevatorConstraint.getClosestToDesired(winchInputs.left.position, getCurrentGoal().getHeight());
        
        Logger.recordOutput("Elevator/desiredPosition", getCurrentGoal().getHeight());
        winch.setScorePosition(getCurrentGoal().getHeight());
        break;
    }

    Logger.recordOutput("Elevator/currentState", getCurrentGoal());

    Logger.recordOutput("Elevator/Tuning/DesiredHeightInches", desiredTuningHeight.in(Inches));
    Logger.recordOutput("Elevator/Tuning/CurrentHeightInches", getPosition().in(Inches));

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the elevator. */
  public void setGoal(Goal goal, LinearConstraint<DistanceUnit, Distance> constraint) {
    currentGoal = goal;
    elevatorConstraint = constraint;
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
}

package frc.robot.subsystems.elevator;


import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotTime;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

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

  /**
   * Constructs elevator :)
   *
   * @param winch
   */
  public Elevator(WinchIO winch) {
    this.winch = winch;
    winch.updateInputs(winchInputs);
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    Logger.recordOutput(getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    winch.updateInputs(winchInputs);

    switch (getCurrentState()) {
      case CLIMB:
        winch.setClimbPosition(elevatorConstraint.getClosestToDesired(getPosition(), currentState.height));
        break;
      default:
        winch.setScorePosition(elevatorConstraint.getClosestToDesired(getPosition(), currentState.height));
        break;
    }
  }

  /** Sets the goal of the elevator. */
  public void setGoal(State state, LinearConstraint<DistanceUnit, Distance> constraint) {
    currentState = state;
    elevatorConstraint = constraint;
  }

  public Distance getPosition() {
    return winch.getPosition();
  }
}

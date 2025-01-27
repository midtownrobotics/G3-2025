package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.Constraint;
import java.util.List;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {


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
      this(-1);
    }

    /** State has meter height value associated or -1 */
    private State(int height) {
      this.height = Units.Meters.of(height);
    }
  }

  private @Getter @Setter State currentState = State.STOW;

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

    switch (currentState) {
      case CLIMB:
        winch.setClimbPosition(getAllowedPosition(currentState.height));
        break;
      default:
        winch.setScorePosition(getAllowedPosition(currentState.height));
        break;
    }

  }

  /** Sets the goal of the subsystem. */
  public void setGoal(State state, List<Constraint<Distance>> constraints) {
    currentState = state;
    // elevatorConstraints = constraints;
  }

  private Distance getAllowedPosition(Distance target) {
    // for (Constraint<Distance> elevatorConstraint : elevatorConstraints) {
    //   target = elevatorConstraint.apply(target, getPosition());
    // }
    return target;
  }

  public Distance getPosition() {
    return winch.getPosition();
  }
}

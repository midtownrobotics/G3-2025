package frc.robot.subsystems.elevator;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchInputsAutoLogged;
import frc.robot.utils.Constraints.Constraint;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends SubsystemBase {

  private List<Constraint<Distance>> elevatorConstraints = new ArrayList<>();

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
    TUNING(0),
    MANUAL(0);

    private @Getter Distance height;

    /**
     * State has meter height value associated
     *
     * @param height assign -1 if no height associated
     */
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

    switch (getCurrentState()) {

    }
  }

  public Distance getPosition() {
    return winch.getPosition();
  }

  public void set
}

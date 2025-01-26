package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchInputsAutoLogged;
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

  private @Getter @Setter State state;
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

    switch (getState()) {
      case ALGAE_GROUND:
        // getWinch().setPosition(ALGAE_GROUND)
        break;
      case BARGE:
        // getWinch().setPosition(BARGE)
        break;
      case CLIMB:
        break;
      case L1:
        // getWinch().setPosition(L1);
        break;
      case L2:
        // getWinch().setPosition(L2);
        break;
      case L3:
        // getWinch().setPosition(L3);
        break;
      case L4:
        // getWinch().setPosition(L4);
        break;
      case MANUAL:
        break;
      case PROCESSOR:
        // getWinch().setPosition(PROCESSOR);
        break;
      case STATION:
        // getWinch().setPosition(STATION);
        break;
      case TUNING:
        break;
      default:
      case STOW:
        // getWinch().setPosition(STOW);
        break;
    }
  }

  public Distance getPosition() {
    return null;
  }
}

package frc.robot.subsystems.algae_claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.RobotTime;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
  public enum State {
    STOW,
    VOMIT,
    START_POSITION,
    GROUND_INTAKE,
    REEF_INTAKE,
    BARGE_SHOOT_FRONT,
    BARGE_SHOOT_BACK,
    PROCESSOR_SHOOT,
    STACKED_ALGAE_INTAKE,
    CLIMB,
    TUNING,
    MANUAL
  }

  private @Getter @Setter State currentState = State.STOW;

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();

    // state switch case

    // record outputs

    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);

    super.periodic();
  }

  // setState()
  // getState()

  // senseAlgae()
}

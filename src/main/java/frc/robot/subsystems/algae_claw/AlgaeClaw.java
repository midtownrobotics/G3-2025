package frc.robot.subsystems.algae_claw;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristInputsAutoLogged;
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

  private WristIO wristIO;

  private WristInputsAutoLogged wristInputs = new WristInputsAutoLogged();

  /** Constructor for algae claw. */
  public AlgaeClaw(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  public Angle getPosition() {
    return wristInputs.wristPosition;
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();

    // state switch case

    // record outputs

    Logger.recordOutput(
        getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);

    super.periodic();
  }
  // senseAlgae()
}

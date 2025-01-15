package frc.robot.subsystems.algae_claw;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.algae_claw.roller.RollerIO;
import frc.robot.subsystems.algae_claw.roller.RollerInputsAutoLogged;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristInputsAutoLogged;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
  public enum State {
    STOW,
    VOMIT,
    START_POSITION,
    GROUND_INTAKE,
    GROUND_VOMIT,
    REEF_INTAKE,
    BARGE_SHOOT_FRONT,
    BARGE_SHOOT_BACK,
    BARGE_PREPARE_FRONT,
    BARGE_PREPARE_BACK,
    PROCESSOR_SHOOT,
    PROCESSOR_PREPARE,
    STACKED_ALGAE_INTAKE,
    STACKED_ALGAE_VOMIT,
    CLIMB,
    TUNING,
    MANUAL
  }

  private @Getter @Setter State currentState = State.STOW;

  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final WristIO wristIO;
  private final WristInputsAutoLogged wristInputs = new WristInputsAutoLogged();

  /** Constructor for algae claw. */
  public AlgaeClaw(RollerIO rollerIO, WristIO wristIO) {
    this.rollerIO = rollerIO;
    this.wristIO = wristIO;
  }

  @AutoLogOutput
  public Angle getPosition() {
    return null;
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();

    Logger.processInputs(getName() + "/roller", rollerInputs);
    Logger.processInputs(getName() + "/wrist", wristInputs);
    rollerIO.updateInputs(rollerInputs);
    wristIO.updateInputs(wristInputs);

    // state switch case

    // record outputs
    Logger.recordOutput(getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  // senseAlgae()
}

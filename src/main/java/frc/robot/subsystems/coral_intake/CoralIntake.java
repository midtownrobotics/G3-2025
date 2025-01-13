package frc.robot.subsystems.coral_intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.coral_intake.belt.BeltIO;
import frc.robot.subsystems.coral_intake.belt.BeltInputsAutoLogged;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.coral_intake.roller.RollerIO;
import frc.robot.subsystems.coral_intake.roller.RollerInputsAutoLogged;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  public enum State {
    STOW,
    START_POSITION,
    GROUND_INTAKE,
    SOURCE_INTAKE,
    GROUND_VOMIT,
    SOURCE_VOMIT,
    HANDOFF,
    REVERSE_HANDOFF,
    STATION_INTAKE,
    CLIMB,
    TUNING,
    MANUAL
  }

  private @Getter @Setter State currentState = State.STOW;

  private final BeltIO beltIO;
  private final BeltInputsAutoLogged beltInputs = new BeltInputsAutoLogged();
  private final PivotIO pivotIO;
  private final PivotInputsAutoLogged pivotInputs = new PivotInputsAutoLogged();
  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

  /**
   * Initializes Coral Intake with IO classes
   * @param beltIO
   * @param pivotIO
   * @param rollerIO
   */
  public CoralIntake(BeltIO beltIO, PivotIO pivotIO, RollerIO rollerIO) {
    this.beltIO = beltIO;
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    beltIO.updateInputs(beltInputs);
    pivotIO.updateInputs(pivotInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(getName() + "/belt", beltInputs);
    Logger.processInputs(getName() + "/pivot", beltInputs);
    Logger.processInputs(getName() + "/roller", beltInputs);

    // state switch case

    Logger.recordOutput(getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  // setState()
  // getState()
}

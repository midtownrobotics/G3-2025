package frc.robot.subsystems.algae_claw;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
  public CircularConstraint wristConstraint = new CircularConstraint();

  public enum Goal {
    STOW(0, 0),
    VOMIT(0, 7),
    START_POSITION(0, 0),
    GROUND_INTAKE(0, -7),
    GROUND_VOMIT(0, 7),
    REEF_INTAKE(0, -7),
    BARGE_SHOOT_FRONT(0, 7),
    BARGE_SHOOT_BACK(0, 7),
    BARGE_PREPARE_FRONT(0, 0),
    BARGE_PREPARE_BACK(0, 0),
    PROCESSOR_SHOOT(0, 7),
    PROCESSOR_PREPARE(0, 0),
    STACKED_ALGAE_INTAKE(0, -7),
    STACKED_ALGAE_VOMIT(0, 7),
    CLIMB(0, 0),
    TUNING(0, 0),
    MANUAL(0, 0);

    @Getter private final Angle angle;
    @Getter private final Voltage rollerVoltage;

    private Goal(double angle, double rollerVoltage) {
      this.angle = Units.Radians.of(angle);
      this.rollerVoltage = Units.Volts.of(rollerVoltage);
    }

  }

  private @Getter Goal currentGoal = Goal.STOW;
  private static final Current kCurrentThreshold = Units.Amps.of(5);

  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final WristIO wristIO;
  private final WristInputsAutoLogged wristInputs = new WristInputsAutoLogged();

  private boolean hasAlgae;

  /** Constructor for algae claw. */
  public AlgaeClaw(RollerIO rollerIO, WristIO wristIO) {
    this.rollerIO = rollerIO;
    this.wristIO = wristIO;
    hasAlgae = false;
  }

  @AutoLogOutput
  public Angle getPosition() {
    return wristInputs.position;
  }

  /**
   * get angle of wrist
   * @return
   */
  public Angle getAngle() {
    return wristInputs.absolutePosition;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    Logger.processInputs(getName() + "/roller", rollerInputs);
    Logger.processInputs(getName() + "/wrist", wristInputs);
    rollerIO.updateInputs(rollerInputs);
    wristIO.updateInputs(wristInputs);

    // goal switch case
    switch (getCurrentGoal()) {
      case BARGE_SHOOT_BACK:
      case BARGE_SHOOT_FRONT:
      case GROUND_INTAKE:
      case GROUND_VOMIT:
      case PROCESSOR_SHOOT:
      case REEF_INTAKE:
      case STACKED_ALGAE_INTAKE:
      case STACKED_ALGAE_VOMIT:
      case VOMIT:
        rollerIO.setVoltage(getCurrentGoal().getRollerVoltage());
        wristIO.setPosition(wristConstraint.getClosestToDesired(wristInputs.position, currentGoal.getAngle()));
        if (hasAlgae) {
          if (rollerInputs.torqueCurrent.gte(kCurrentThreshold)) { hasAlgae = false; }
        }
        break;
      default:
        rollerIO.setVoltage(getCurrentGoal().getRollerVoltage());
        wristIO.setPosition(wristConstraint.getClosestToDesired(wristInputs.position, currentGoal.getAngle()));
        break;
    }

    Logger.recordOutput("AlgaeClaw/currentState", getCurrentGoal());
    Logger.recordOutput("AlgaeClaw/rollerVoltage", getCurrentGoal().getRollerVoltage());
    Logger.recordOutput("AlgaeClaw/wristPosition", getCurrentGoal().getAngle());
    // record outputs
    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(Goal goal, CircularConstraint constraint) {
    currentGoal = goal;
    wristConstraint = constraint;
  }

  /** if has algae */
  public boolean senseAlgae() {
    return hasAlgae;
  }
}

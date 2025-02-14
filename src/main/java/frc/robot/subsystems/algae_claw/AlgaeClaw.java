package frc.robot.subsystems.algae_claw;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
  public CircularConstraint wristConstraint = new CircularConstraint();

  @RequiredArgsConstructor
  public enum Goal { // goals with todo need angles set, others inherit
    STOW(Degrees.of(0), Volts.of(0)), // TODO
    VOMIT(STOW.getAngle(), AlgaeClawConstants.SHOOT_ROLLER_VOLTAGE),
    START_POSITION(Degrees.of(0), Volts.of(0)), // TODO
    GROUND_INTAKE(Degrees.of(0), AlgaeClawConstants.INTAKE_ROLLER_VOLTAGE), // TODO
    GROUND_VOMIT(GROUND_INTAKE.getAngle(), AlgaeClawConstants.SHOOT_ROLLER_VOLTAGE),
    REEF_INTAKE(Degrees.of(0), AlgaeClawConstants.INTAKE_ROLLER_VOLTAGE), // TODO
    BARGE_SHOOT_FRONT(Degrees.of(0), AlgaeClawConstants.SHOOT_ROLLER_VOLTAGE), // TODO
    BARGE_SHOOT_BACK(Degrees.of(0), AlgaeClawConstants.SHOOT_ROLLER_VOLTAGE), // TODO
    BARGE_PREPARE_FRONT(BARGE_SHOOT_FRONT.getAngle(), Volts.of(0)),
    BARGE_PREPARE_BACK(BARGE_SHOOT_BACK.getAngle(), Volts.of(0)),
    PROCESSOR_SHOOT(Degrees.of(0), AlgaeClawConstants.SHOOT_ROLLER_VOLTAGE), // TODO
    PROCESSOR_PREPARE(PROCESSOR_SHOOT.getAngle(), Volts.of(0)),
    STACKED_ALGAE_INTAKE(Degrees.of(0), AlgaeClawConstants.INTAKE_ROLLER_VOLTAGE), // TODO
    STACKED_ALGAE_VOMIT(STACKED_ALGAE_INTAKE.getAngle(), AlgaeClawConstants.SHOOT_ROLLER_VOLTAGE),
    CLIMB(Degrees.of(0), Volts.of(0)),
    TUNING(Degrees.of(0), Volts.of(0)),
    MANUAL(Degrees.of(0), Volts.of(0));

    @Getter private final Angle angle;
    @Getter private final Voltage rollerVoltage;
  }

  // No getter because isHasAlgae() makes no sense
  private boolean hasAlgae = false;

  private @Getter Goal currentGoal = Goal.STOW;

  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final WristIO wristIO;
  private final WristInputsAutoLogged wristInputs = new WristInputsAutoLogged();

  private SysIdRoutine routine;

  /** Constructor for algae claw. */
  public AlgaeClaw(RollerIO rollerIO, WristIO wristIO) {
    this.rollerIO = rollerIO;
    this.wristIO = wristIO;
      SysIdRoutine.Mechanism sysIdMech = new SysIdRoutine.Mechanism(
        wristIO::setVoltage,
        this::motorSysIdLog,
        this
    );

    routine = new SysIdRoutine(new Config(Volts.of(1).per(Second), Volts.of(1), Seconds.of(3)), sysIdMech);
  }

  private void motorSysIdLog(SysIdRoutineLog log) {
    log.motor("wrist")
      .voltage(wristInputs.appliedVoltage)
      .angularPosition(wristInputs.position)
      .angularVelocity(wristInputs.velocity);
  }

  @AutoLogOutput
  public Angle getPosition() {
    return wristInputs.position;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    Logger.processInputs(getName() + "/roller", rollerInputs);
    Logger.processInputs(getName() + "/wrist", wristInputs);
    rollerIO.updateInputs(rollerInputs);
    wristIO.updateInputs(wristInputs);

    Voltage desiredRollerVoltage = getCurrentGoal().getRollerVoltage();

    // Goal switch case
    switch (getCurrentGoal()) {
      case BARGE_SHOOT_BACK:
      case BARGE_SHOOT_FRONT:
      case GROUND_VOMIT:
      case PROCESSOR_SHOOT:
      case STACKED_ALGAE_VOMIT:
      case VOMIT:
        hasAlgae = false;
        break;
      case GROUND_INTAKE:
      case STACKED_ALGAE_INTAKE:
      case REEF_INTAKE:
        if (wristInputs.torqueCurrent.gt(AlgaeClawConstants.CURRENT_INTAKE_MAXIMUM_DETECTION)) {
          hasAlgae = true;
        }
        break;
      default:
        break;
    }

    if (hasAlgae && desiredRollerVoltage == Volts.of(0)) {
      desiredRollerVoltage = AlgaeClawConstants.HOLD_PIECE_ROLLER_VOLTAGE;
      if (wristInputs.velocity.gt(AlgaeClawConstants.MIN_ANGULAR_VELOCITY_PIECE_DETECTION)) {
        hasAlgae = false;
      }
    }

    rollerIO.setVoltage(desiredRollerVoltage);

    if (getCurrentGoal() != Goal.TUNING) {
      wristIO.setPosition(wristConstraint.getClosestToDesired(wristInputs.position, currentGoal.getAngle()));
    }

    Logger.recordOutput("AlgaeClaw/currentState", getCurrentGoal());
    Logger.recordOutput("AlgaeClaw/rollerVoltage", desiredRollerVoltage);
    Logger.recordOutput("AlgaeClaw/wristPosition", getCurrentGoal().getAngle());
    // record outputs
    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(Goal goal, CircularConstraint constraint) {
    currentGoal = goal;
    wristConstraint = constraint;
  }

  /** If {@link currentAlgaeGamePieceState} is {@code HOLDING}. */
  public boolean hasAlgae() {
    return hasAlgae;
  }

  /**
   * Runs the sysIdQuasistatic test on the algae claw.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Runs the sysIdDynamic test on the algae claw.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}

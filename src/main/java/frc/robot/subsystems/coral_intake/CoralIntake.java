package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
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
import frc.robot.sensors.Photoelectric;
import frc.robot.subsystems.coral_intake.belt.BeltIO;
import frc.robot.subsystems.coral_intake.belt.BeltInputsAutoLogged;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {

  public LinearConstraint<AngleUnit, Angle> coralIntakeConstraint = new LinearConstraint<AngleUnit, Angle>(
      CoralIntakeConstants.coralIntakeMinAngle, CoralIntakeConstants.coralIntakeMaxAngle);

  public enum Goal {
    GROUND_INTAKE(Degrees.of(0), Volts.of(7)),
    GROUND_VOMIT(GROUND_INTAKE.getAngle(), Volts.of(-7)),
    STATION_INTAKE(Degrees.of(0), Volts.of(0)),
    HANDOFF(Degrees.of(120), Volts.of(0)),
    HANDOFF_ADJUSTING(HANDOFF.getAngle(), Volts.of(0), Volts.of(7)),
    STOW(HANDOFF.getAngle(), Volts.of(0)),
    CLIMB(Degrees.of(45), Volts.of(0)),
    TUNING(Degrees.of(0), Volts.of(0)),
    MANUAL(Degrees.of(0), Volts.of(0));

    private @Getter Angle angle;
    private @Getter Voltage rollerVoltage;
    private @Getter Voltage beltVoltage = Volts.of(0);

    /**
     * Goal has angle, and rollerVoltage associated.
     * 
     * @param angle
     * @param rollerVoltage
     */
    private Goal(Angle angle, Voltage rollerVoltage) {
      this.angle = angle;
      this.rollerVoltage = rollerVoltage;
    }

    /**
     * Goal has angle, rollerVoltage, and beltVoltage associated.
     * 
     * @param angle
     * @param rollerVoltage
     * @param beltVoltage
     */
    private Goal(Angle angle, Voltage rollerVoltage, Voltage beltVoltage) {
      this(angle, rollerVoltage);
      this.beltVoltage = beltVoltage;
    }
  }

  private @Getter Goal currentGoal = Goal.STOW;

  private final Photoelectric handoffSensor;
  private final Photoelectric centerSensor;

  private final RollerIO beltIO;
  private final RollerInputsAutoLogged beltInputs = new RollerInputsAutoLogged();
  private final PivotIO pivotIO;
  private final PivotInputsAutoLogged pivotInputs = new PivotInputsAutoLogged();
  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

  private SysIdRoutine routine;

  /**
   * Initializes Coral Intake with IO classes
   * 
   * @param beltIO
   * @param pivotIO
   * @param rollerIO
   */
  public CoralIntake(RollerIO beltIO, PivotIO pivotIO, RollerIO rollerIO, Photoelectric centerSensor,
      Photoelectric handoffSensor) {

    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.beltIO = beltIO;
    this.centerSensor = centerSensor;
    this.handoffSensor = handoffSensor;

    SysIdRoutine.Mechanism sysIdMech = new SysIdRoutine.Mechanism(
        pivotIO::setVoltage,
        this::motorSysIdLog,
        this);

    routine = new SysIdRoutine(new Config(Volts.of(1).per(Second), Volts.of(1), Seconds.of(3)), sysIdMech);
  }

  private void motorSysIdLog(SysIdRoutineLog log) {
    log.motor("pivotMotor")
        .voltage(pivotInputs.appliedVoltage)
        .angularPosition(pivotInputs.absolutePosition)
        .angularVelocity(pivotInputs.velocity);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    pivotIO.updateInputs(pivotInputs);
    rollerIO.updateInputs(rollerInputs);

    Logger.processInputs(getName() + "/belt", beltInputs);
    Logger.processInputs(getName() + "/pivot", pivotInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());

    Voltage desiredBeltVoltage = currentGoal.getBeltVoltage();
    Voltage desiredRollerVoltage = currentGoal.getRollerVoltage();
    Angle desiredAngle = currentGoal.getAngle();

    // Goal switch case

    switch (getCurrentGoal()) {
      case HANDOFF:
        pivotIO.setPosition(desiredAngle);
        rollerIO.setVoltage(desiredRollerVoltage);
        beltIO.setVoltage(desiredBeltVoltage);
        break;
      case HANDOFF_ADJUSTING:
        pivotIO.setPosition(desiredAngle);
        rollerIO.setVoltage(desiredRollerVoltage);
        if (isCoralBlockingMovement()) {
          beltIO.setVoltage(desiredBeltVoltage);
        } else {
          beltIO.setVoltage(desiredRollerVoltage.times(-1));
        }
      case TUNING:
        break;
      default:
        pivotIO.setPosition(desiredAngle);
        beltIO.setVoltage(desiredBeltVoltage);
        rollerIO.setVoltage(desiredRollerVoltage);
        break;
    }
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(Goal goal, LinearConstraint<AngleUnit, Angle> constraint) {
    currentGoal = goal;
    coralIntakeConstraint = constraint;
  }

  public Angle getPivotPosition() {
    return pivotInputs.offsetedPosition;
  }

  private double coralLastDetected;

  /**
   * Uses photoelectric sensors to detect coral.
   * @return {@code boolean} Wheter coral is detected.
   */
  public boolean isCoralDetected() {
    boolean value = handoffSensor.isTriggered() || centerSensor.isTriggered();
    if (value) {
      coralLastDetected = Timer.getFPGATimestamp();
    }
    if (Timer.getFPGATimestamp() - coralLastDetected >= CoralIntakeConstants.coralDetectionIdleDelay.in(Seconds)) {
      return true;
    }
    return value;
  }

  /**
   * Uses photoelectric sensors to detect if coral needs to be adjusted to be centered.
   * @return {@code boolean} Whether coral needs adjusting.
   */
  public boolean doesCoralNeedAdjusting() {
    return handoffSensor.isTriggered() != centerSensor.isTriggered();
  }

  /**
   * Uses photoelectric sensors to detect if coral will block the intake from moving into handoff mode.
   * @return {@code boolean} Wheter coral is blocking movement.
   */
  public boolean isCoralBlockingMovement() {
    return handoffSensor.isTriggered() && !centerSensor.isTriggered();
  }

  /**
   * Runs the sysIdQuasistatic test on the coral intake.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Runs the sysIdDynamic test on the coral intake.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}

package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.lib.dashboard.LoggedDigitalInput;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  public SlewRateLimiter voltageRateLimiter = new SlewRateLimiter(10);

  public LinearConstraint<AngleUnit, Angle> coralIntakeConstraint = new LinearConstraint<AngleUnit, Angle>(
      CoralIntakeConstants.coralIntakeMinAngle, CoralIntakeConstants.coralIntakeMaxAngle);

  public enum Goal {
    STOW(Degrees.of(84), Volts.of(0.75), Volts.of(1)),
    GROUND_INTAKE(Degrees.of(-14), Volts.of(12), Volts.of(1)),
    GROUND_VOMIT(GROUND_INTAKE.getAngle(), Volts.of(-5)),
    STATION_VOMIT(Degrees.of(101.5), Volts.of(-5)),
    STATION_INTAKE(STATION_VOMIT.getAngle(), Volts.of(10), Volts.of(3)),
    HANDOFF(Degrees.of(135), Volts.of(1.0), Volts.of(-4)),
    PRE_HANDOFF_ADJUST_CORAL(Degrees.of(100), Volts.of(12), Volts.of(3.5)),
    CLIMB(Degrees.of(82), Volts.of(0)),
    CLIMB_BOTTOM(Degrees.of(104), Volts.of(0)),
    L1(Degrees.of(74), Volts.of(-4)),
    L1_PREPARE(STOW.getAngle(), Volts.zero()),
    ALGAE_INTAKE(Degrees.of(39), Volts.of(-9.5)),
    HOLD_ALGAE(Degrees.of(49), Volts.of(-1)),
    ALGAE_SHOOT(Degrees.of(49), Volts.of(10)),
    ZERO(),
    TUNING(),
    MANUAL();

    private @Getter Angle angle;
    private @Getter Voltage rollerVoltage;
    private @Getter Voltage beltVoltage = Volts.of(0);

    /**
     * Goal has angle and rollerVoltage associated.
     *
     * @param angle
     * @param rollerVoltage
     */
    private Goal(Angle angle, Voltage rollerVoltage) {
      this.angle = angle;
      this.rollerVoltage = rollerVoltage;
      this.beltVoltage = Volts.zero();
    }

    private Goal() {
    };

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

  public final Trigger atGoalTrigger = new Trigger(this::atGoal);
  public final Trigger handoffSensorTrigger;
  public final Trigger centerSensorTrigger;
  public final Trigger pieceDetectedTrigger;
  private final Trigger pieceWillCollideTrigger;

  private @Getter Goal currentGoal = Goal.STOW;

  private final LoggedDigitalInput handoffSensor;
  private final LoggedDigitalInput centerSensor;
  private final LoggedDigitalInput upperZeroSensor;
  private final LoggedDigitalInput lowerZeroSensor;

  private final Debouncer zeroSensorDebouncer = new Debouncer(5, DebounceType.kRising);

  private final RollerIO beltIO;
  private final RollerInputsAutoLogged beltInputs = new RollerInputsAutoLogged();
  private final PivotIO pivotIO;
  private final PivotInputsAutoLogged pivotInputs = new PivotInputsAutoLogged();
  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

  private ArmFeedforward pivotFeedforward = new ArmFeedforward(
      CoralIntakeConstants.PID.s.get(), CoralIntakeConstants.PID.g.get(), CoralIntakeConstants.PID.v.get());

  private ProfiledPIDController pidController = new ProfiledPIDController(CoralIntakeConstants.PID.p.get(),
      CoralIntakeConstants.PID.i.get(), CoralIntakeConstants.PID.d.get(),
      new Constraints(CoralIntakeConstants.PID.maxPivotV.get(), CoralIntakeConstants.PID.maxPivotA.get()));

  private SysIdRoutine routine;

  /**
   * Initializes Coral Intake with IO classes
   *
   * @param beltIO
   * @param pivotIO
   * @param rollerIO
   */
  public CoralIntake(RollerIO beltIO, PivotIO pivotIO, RollerIO rollerIO, LoggedDigitalInput centerSensor,
      LoggedDigitalInput handoffSensor, LoggedDigitalInput upperZeroSensor, LoggedDigitalInput lowerZeroSensor) {

    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.beltIO = beltIO;
    this.centerSensor = centerSensor;
    this.handoffSensor = handoffSensor;

    this.upperZeroSensor = upperZeroSensor;
    this.lowerZeroSensor = lowerZeroSensor;

    this.handoffSensorTrigger = new Trigger(handoffSensor::isTriggered);
    this.centerSensorTrigger = new Trigger(centerSensor::isTriggered);
    this.pieceDetectedTrigger = handoffSensorTrigger.or(centerSensorTrigger);
    this.pieceWillCollideTrigger = handoffSensorTrigger.and(centerSensorTrigger.negate());

    pidController.setIntegratorRange(-0.2, 0.2);
    pidController.setIZone(0.2);

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

  /** Gets the debounced value of the sensors. */
  public boolean getZeroSensorDebounced(boolean upper) {
    return zeroSensorDebouncer.calculate(upper ? upperZeroSensor.isTriggered() : lowerZeroSensor.isTriggered());
  }

  private LoggedTunableNumber tuningDesiredAngle = new LoggedTunableNumber("CoralIntake/desiredAngle", 0.0);

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    if (RobotState.isDisabled()) {
      double position = getPosition().in(Radians);
      pidController.setGoal(position);
      pidController.reset(position);
    }

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      pidController.setConstraints(new TrapezoidProfile.Constraints(CoralIntakeConstants.PID.maxPivotV.get(),
          CoralIntakeConstants.PID.maxPivotA.get()));
    }, CoralIntakeConstants.PID.maxPivotA, CoralIntakeConstants.PID.maxPivotV);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      pivotFeedforward = new ArmFeedforward(CoralIntakeConstants.PID.s.get(), CoralIntakeConstants.PID.g.get(),
          CoralIntakeConstants.PID.v.get());
    }, CoralIntakeConstants.PID.s, CoralIntakeConstants.PID.v, CoralIntakeConstants.PID.g);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      pidController.setPID(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(),
          CoralIntakeConstants.PID.d.get());
    }, CoralIntakeConstants.PID.p, CoralIntakeConstants.PID.i, CoralIntakeConstants.PID.d);

    beltIO.updateInputs(beltInputs);
    pivotIO.updateInputs(pivotInputs);
    rollerIO.updateInputs(rollerInputs);

    Logger.processInputs(getName() + "/belt", beltInputs);
    Logger.processInputs(getName() + "/pivot", pivotInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);

    // currentGoal = Goal.CLIMB;

    Voltage desiredBeltVoltage = Volts.of(voltageRateLimiter.calculate(currentGoal.getBeltVoltage().in(Volts)));
    // if (desiredBeltVoltage.equals(Volts.zero()) &&
    // pieceDetectedTrigger.getAsBoolean()) {
    // if (getPosition().lte(Degrees.of(135))) {
    // if (!centerSensor.isTriggered()) {
    // desiredBeltVoltage = Volts.of(4);
    // }
    // } else {
    // desiredBeltVoltage = Volts.of(-3);
    // }
    // }

    // currentGoal = Goal.TUNING;

    Voltage desiredRollerVoltage = currentGoal.getRollerVoltage();
    Angle desiredAngle = currentGoal.getAngle();

    Angle constrainedAngle = coralIntakeConstraint.getClampedValue(desiredAngle);

    if (Constants.tuningMode.get()) {
      constrainedAngle = Degrees.of(tuningDesiredAngle.get());
    }

    switch (getCurrentGoal()) {
      case ZERO:
        if (!getZeroSensorDebounced(true)) {
          pivotIO.setVoltage(Volts.of(-1));
          break;
        }
        pivotIO.zeroPivotAngle(CoralIntakeConstants.upperSensorTriggeredAngle);
        pivotIO.setVoltage(Volts.zero());
        break;
      case HANDOFF:
        if (getPosition().lt(Degrees.of(131))) {
          desiredBeltVoltage = Goal.PRE_HANDOFF_ADJUST_CORAL.getBeltVoltage();
        } else {
          desiredBeltVoltage = Goal.HANDOFF.getBeltVoltage();
        }

        if (getPosition().isNear(desiredAngle, Degrees.of(3))) {
          pivotIO.setVoltage(Volts.of(-0.1));
        } else {
          pivotIO.setVoltage(calculateVoltageForPosition(constrainedAngle));
        }

        beltIO.setVoltage(desiredBeltVoltage);
        rollerIO.setVoltage(desiredRollerVoltage);
        break;
      case L1:
        pivotIO.setVoltage(calculateVoltageForPosition(constrainedAngle));
        beltIO.setVoltage(desiredBeltVoltage);
        if (atGoal()) {
          rollerIO.setVoltage(desiredRollerVoltage);
        }
        break;
      case GROUND_INTAKE:
        if (getPosition().isNear(desiredAngle, Degrees.of(3)) || getPosition().lt(desiredAngle)) {
          pivotIO.setVoltage(Volts.of(-0.2));
        } else {
          pivotIO.setVoltage(calculateVoltageForPosition(constrainedAngle));
        }
        beltIO.setVoltage(desiredBeltVoltage);
        rollerIO.setVoltage(desiredRollerVoltage);
        break;
      default:
        pivotIO.setVoltage(calculateVoltageForPosition(constrainedAngle));
        beltIO.setVoltage(desiredBeltVoltage);
        rollerIO.setVoltage(desiredRollerVoltage);
        break;
    }

    Logger.recordOutput("CoralIntake/currentGoal", getCurrentGoal());
    Logger.recordOutput("CoralIntake/goalAngle", desiredAngle);
    Logger.recordOutput("CoralIntake/atGoal", atGoal());

    Logger.recordOutput("CoralIntake/currentAngle", getPosition());
    Logger.recordOutput("CoralIntake/currentVelocity", getVelocity());

    Logger.recordOutput("CoralIntake/constraintMax", coralIntakeConstraint.getUpper());
    Logger.recordOutput("CoralIntake/constraintMin", coralIntakeConstraint.getLower());
    Logger.recordOutput("CoralIntake/constrainedGoalAngle", constrainedAngle);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral intake pivot. */
  public void setGoal(Goal goal) {
    currentGoal = goal;
  }

  /** Sets the constraints of the coral intake pivot. */
  public void setConstraints(LinearConstraint<AngleUnit, Angle> constraint) {
    coralIntakeConstraint = constraint;
  }

  private double coralLastDetected;

  /**
   * Uses photoelectric sensors to detect coral.
   *
   * @return {@code boolean} Wheter coral is detected.
   */
  public boolean isCoralDetected() {
    if (handoffSensor.isTriggered() || centerSensor.isTriggered()) {
      coralLastDetected = Timer.getFPGATimestamp();
    }
    if (Timer.getFPGATimestamp() - coralLastDetected >= CoralIntakeConstants.coralDetectionIdleDelay.in(Seconds)) {
      return false;
    }
    return true;
  }

  /**
   * Uses photoelectric sensors to detect if coral needs to be adjusted to be
   * centered.
   *
   * @return {@code boolean} Whether coral needs adjusting.
   */
  public boolean doesCoralNeedAdjusting() {
    return handoffSensor.isTriggered() != centerSensor.isTriggered();
  }

  /**
   * Uses photoelectric sensors to detect if coral will block the intake from
   * moving into handoff mode.
   *
   * @return {@code boolean} Wheter coral is triggering handoff sensor but not
   *         center sensor.
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

  public Angle getPosition() {
    return pivotInputs.absolutePosition;
  }

  public AngularVelocity getVelocity() {
    return pivotInputs.velocity;
  }

  private Voltage calculateVoltageForPosition(Angle desired) {
    Voltage pidVoltage = Volts.of(pidController.calculate(getPosition().in(Radians), desired.in(Radians)));

    TrapezoidProfile.State setpoint = pidController.getSetpoint();
    Voltage ffVoltage = Volts.of(pivotFeedforward.calculate(setpoint.position, setpoint.velocity));

    Voltage totalVoltage = pidVoltage.plus(ffVoltage);

    Logger.recordOutput("CoralIntake/AnglePID/goalPosition", desired);
    Logger.recordOutput("CoralIntake/AnglePID/setpointPosition", setpoint.position);
    Logger.recordOutput("CoralIntake/AnglePID/setpointVelocity", setpoint.velocity);

    Logger.recordOutput("CoralIntake/AnglePID/pidVoltage", pidVoltage);
    Logger.recordOutput("CoralIntake/AnglePID/ffVoltage", ffVoltage);
    Logger.recordOutput("CoralIntake/AnglePID/desiredPivotVoltage", totalVoltage);

    return totalVoltage;
  }

  /**
   * Returns true if the intake is within a small threshold distance to the goal.
   */
  public boolean atGoal() {
    return atGoal(getCurrentGoal());
  }

  /**
   * Returns true if the intake is within a small threshold distance to the
   * specified goal.
   */
  public boolean atGoal(Goal goal) {
    return atGoal(goal, Degrees.of(1.0));
  }

  /**
   * Returns true if the intake is within a small threshold distance to the
   * specified goal.
   */
  public boolean atGoal(Goal goal, Angle angleTolerance) {
    return getCurrentGoal() == goal && getPosition().isNear(goal.getAngle(), angleTolerance);
  }

  /**
   * Returns true if the intake is within a small threshold distance to the
   * specified goal.
   */
  public boolean atGoal(Angle angleTolerance) {
    return atGoal(getCurrentGoal(), angleTolerance);
  }

  /**
   * Returns a trigger for if the intake is within a small threshold distance to
   * the goal.
   */
  public Trigger atGoalTrigger(Goal goal) {
    return new Trigger(() -> atGoal(goal));
  }

  /**
   * Returns a trigger for if the intake is within a small threshold distance to
   * the goal.
   */
  public Trigger atGoalTrigger(Goal goal, Angle angleTolerance) {
    return new Trigger(() -> atGoal(goal, angleTolerance));
  }

  /**
   * Returns a command that sets the goal of the intake and finishes immediately.
   */
  public Command setGoalCommand(Goal goal) {
    return runOnce(() -> setGoal(goal));
  }

  /**
   * Returns a command that sets the goal of the intake and then sets it to the
   * endGoal when the command ends.
   */
  public Command setGoalEndCommand(Goal goal, Goal endGoal) {
    return run(() -> setGoal(goal)).finallyDo(() -> setGoal(endGoal));
  }

  /**
   * Returns a command that sets the goal of the intake and waits until it is at
   * the goal.
   */
  public Command setGoalAndWait(Goal goal) {
    return run(() -> setGoal(goal)).until(this::atGoal);
  }

  /**
   * Returns a command that sets the goal of the intake and waits until it is at
   * the goal.
   */
  public Command setGoalAndWait(Goal goal, Angle tolerance) {
    return run(() -> setGoal(goal)).until(() -> atGoal(tolerance));
  }
}

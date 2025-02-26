package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.lib.LoggedTunableNumber;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.robot.sensors.Photoelectric;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;

public class CoralIntake extends SubsystemBase {

  public LinearConstraint<AngleUnit, Angle> coralIntakeConstraint = new LinearConstraint<AngleUnit, Angle>(
      CoralIntakeConstants.coralIntakeMinAngle, Radians.of(CoralIntakeConstants.coralIntakeMaxAngle.get()));

  public enum Goal {
    STOW(Degrees.of(137), Volts.of(0)),
    GROUND_INTAKE(Degrees.of(-9), Volts.of(12)),
    GROUND_VOMIT(GROUND_INTAKE.getAngle(), Volts.of(-7)),
    STATION_INTAKE(STOW.getAngle(), Volts.of(4)),
    HANDOFF(STOW.getAngle(), Volts.of(0)),
    HANDOFF_PUSH_CORAL(STOW.getAngle(), Volts.of(-1)),
    HANDOFF_ADJUSTING(HANDOFF.getAngle(), Volts.of(-1), Volts.of(7)),
    CLIMB(Degrees.of(65), Volts.of(0)),
    TUNING(Degrees.of(0), Volts.of(0)),
    MANUAL(Degrees.of(0), Volts.of(0));

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

  public final Trigger atGoalTrigger = new Trigger(this::atGoal);
  public final Trigger handoffSensorTrigger;
  public final Trigger centerSensorTrigger;
  public final Trigger pieceDetectedTrigger;
  private final Trigger pieceWillCollideTrigger;

  private @Getter Goal currentGoal = Goal.STOW;

  private final Photoelectric handoffSensor;
  private final Photoelectric centerSensor;

  private final RollerIO beltIO;
  private final RollerInputsAutoLogged beltInputs = new RollerInputsAutoLogged();
  private final PivotIO pivotIO;
  private final PivotInputsAutoLogged pivotInputs = new PivotInputsAutoLogged();
  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

    private ArmFeedforward pivotFeedforward = new ArmFeedforward(
        CoralIntakeConstants.PID.s.get(), CoralIntakeConstants.PID.g.get(), CoralIntakeConstants.PID.v.get());

    private ProfiledPIDController pidController = new ProfiledPIDController(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(), CoralIntakeConstants.PID.d.get(), new Constraints(CoralIntakeConstants.PID.maxPivotV.get(), CoralIntakeConstants.PID.maxPivotA.get()));

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

    this.handoffSensorTrigger = new Trigger(handoffSensor::isTriggered);
    this.centerSensorTrigger = new Trigger(centerSensor::isTriggered);
    this.pieceDetectedTrigger = handoffSensorTrigger.or(centerSensorTrigger);
    this.pieceWillCollideTrigger = handoffSensorTrigger.and(centerSensorTrigger.negate());

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
    pidController.setPID(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(), CoralIntakeConstants.PID.d.get());
}, CoralIntakeConstants.PID.p, CoralIntakeConstants.PID.i, CoralIntakeConstants.PID.d);

    beltIO.updateInputs(beltInputs);
    pivotIO.updateInputs(pivotInputs);
    rollerIO.updateInputs(rollerInputs);

    Logger.processInputs(getName() + "/belt", beltInputs);
    Logger.processInputs(getName() + "/pivot", pivotInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);

    // currentGoal = Goal.CLIMB;

    Voltage desiredBeltVoltage = currentGoal.getBeltVoltage();
    if (desiredBeltVoltage.equals(Volts.zero()) && pieceDetectedTrigger.getAsBoolean()) {
      if (getPosition().lte(Degrees.of(135))) {
        if (!centerSensor.isTriggered()) {
          desiredBeltVoltage = Volts.of(4);
        } 
      } else {
          desiredBeltVoltage = Volts.of(-3);
      }
    }

    Voltage desiredRollerVoltage = currentGoal.getRollerVoltage();
    Angle desiredAngle = currentGoal.getAngle();

    if (Constants.tuningMode.get()) {
      desiredAngle = Degrees.of(tuningDesiredAngle.get());
    }

    switch (getCurrentGoal()) {
      case HANDOFF_ADJUSTING:
        pivotIO.setVoltage(calculateVoltageForPosition(desiredAngle));
        rollerIO.setVoltage(desiredRollerVoltage);
        if (isCoralBlockingMovement()) {
          beltIO.setVoltage(desiredBeltVoltage);
        } else {
          beltIO.setVoltage(desiredRollerVoltage.times(-1));
        }
      // case TUNING:
      //   desiredAngle = tuningDesiredAngle;
      //   desiredRollerVoltage = tuningDesiredRollerVoltage;
      //   pivotIO.setPosition(desiredAngle);
      //   beltIO.setVoltage(desiredBeltVoltage);
      //   rollerIO.setVoltage(desiredRollerVoltage);
      //   break;
      default:
        pivotIO.setVoltage(calculateVoltageForPosition(desiredAngle));
        beltIO.setVoltage(desiredBeltVoltage);
        rollerIO.setVoltage(desiredRollerVoltage);
        break;
    }

    Logger.recordOutput("CoralIntake/currentState", getCurrentGoal());
    Logger.recordOutput("CoralIntake/desiredAngle", desiredAngle);
    Logger.recordOutput("CoralIntake/desiredBeltVoltage", desiredBeltVoltage);
    Logger.recordOutput("CoralIntake/desiredRollerVoltage", desiredRollerVoltage);

    Logger.recordOutput("CoralIntake/AngleTuning/g_max", CoralIntakeConstants.g_max);
    Logger.recordOutput("CoralIntake/AngleTuning/g_min", CoralIntakeConstants.g_min);
    Logger.recordOutput("CoralIntake/AngleTuning/e_max", CoralIntakeConstants.e_max);
    Logger.recordOutput("CoralIntake/AngleTuning/e_min", CoralIntakeConstants.e_min);
    Logger.recordOutput("CoralIntake/AngleTuning/breakpoint", CoralIntakeConstants.breakPoint);
    Logger.recordOutput("CoralIntake/AngleTuning/zeroOffset", CoralIntakeConstants.zeroOffset);
    Logger.recordOutput("CoralIntake/AngleTuning/g_position", getPosition());
    Logger.recordOutput("CoralIntake/AngleTuning/g_velocity", getVelocity());
    

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(Goal goal, LinearConstraint<AngleUnit, Angle> constraint) {
    currentGoal = goal;
    coralIntakeConstraint = constraint;
  }

  public Angle getPivotPosition() {
    return pivotInputs.position;
  }

  private double coralLastDetected;

  /**
   * Uses photoelectric sensors to detect coral.
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
   * Uses photoelectric sensors to detect if coral needs to be adjusted to be centered.
   * @return {@code boolean} Whether coral needs adjusting.
   */
  public boolean doesCoralNeedAdjusting() {
    return handoffSensor.isTriggered() != centerSensor.isTriggered();
  }

  /**
   * Uses photoelectric sensors to detect if coral will block the intake from moving into handoff mode.
   * @return {@code boolean} Wheter coral is triggering handoff sensor but not center sensor.
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

  /**
   * incrementTuningAngle
   * @return
   */
  // public Command incrementTuningAngle() {
  //   return new InstantCommand(() -> tuningDesiredAngle.plus(CoralIntakeConstants.tuningAngleIncrementDecrementAmmount), this);
  // }

  /**
   * decrementTuningAngle
   * @return
   */
  // public Command decrementTuningAngle() {
  //   return new InstantCommand(() -> tuningDesiredAngle.minus(CoralIntakeConstants.tuningAngleIncrementDecrementAmmount), this);
  // }

  private Angle getPosition() {
    Angle continuousEncoderPosition = pivotInputs.absolutePosition;
    // if (continuousEncoderPosition.gt(CoralIntakeConstants.breakPoint)) {
    //   continuousEncoderPosition = continuousEncoderPosition.minus(Rotations.of(1));
    // }

    if (continuousEncoderPosition.lt(CoralIntakeConstants.breakPoint)) {
      continuousEncoderPosition = continuousEncoderPosition.plus(Rotations.of(1));
    }
    // if ()
    // Angle continuousEncoderPosition = UnitUtil
    //     .normalize(Units.Rotations.of(encoder.get()).minus(CoralIntakeConstants.breakPoint))
    //     .plus(CoralIntakeConstants.breakPoint);
    return continuousEncoderPosition.times(-0.5).plus(CoralIntakeConstants.zeroOffset);

  }

  private AngularVelocity getVelocity() {
    return pivotInputs.velocity.unaryMinus();
  }

  private Voltage calculateVoltageForPosition(Angle desired) {
    Voltage pidVoltage = Volts.of(pidController.calculate(getPosition().in(Radians), desired.in(Radians)));

    TrapezoidProfile.State setpoint = pidController.getSetpoint();
    Voltage ffVoltage = Volts.of(pivotFeedforward.calculate(setpoint.position, setpoint.velocity));

    Voltage totalVoltage = pidVoltage.plus(ffVoltage);

    Logger.recordOutput("CoralIntake/AnglePID/givenPosition", desired);
    Logger.recordOutput("CoralIntake/AnglePID/setpointPosition", setpoint.position);
    Logger.recordOutput("CoralIntake/AnglePID/pidVoltage", pidVoltage);
    Logger.recordOutput("CoralIntake/AnglePID/ffVoltage", ffVoltage);
    Logger.recordOutput("CoralIntake/AnglePID/desiredPivotVoltage", totalVoltage);

    return totalVoltage;
  }

  public boolean atGoal() {
    return atGoal(getCurrentGoal());
  }

  public boolean atGoal(Goal goal) {
    return getCurrentGoal() == goal && getPivotPosition().isNear(goal.getAngle(), Degrees.of(0.5));
  }

  public Trigger atGoalTrigger(Goal goal) {
    return new Trigger(() -> atGoal(goal));
  }
}

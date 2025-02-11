package frc.robot.subsystems.coral_intake;

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
import org.opencv.photo.Photo;

public class CoralIntake extends SubsystemBase {

  public LinearConstraint<AngleUnit, Angle> coralIntakeConstraint = new LinearConstraint<AngleUnit, Angle>(CoralIntakeConstants.coralIntakeMinAngle, CoralIntakeConstants.coralIntakeMaxAngle);

  public enum Goal {
    GROUND_INTAKE(0,7),
    GROUND_VOMIT(-7, GROUND_INTAKE),
    STATION_INTAKE(0, 0),
    HANDOFF(120,0),
    STOW(HANDOFF),
    REVERSE_HANDOFF(HANDOFF),
    CLIMB(45,0),
    TUNING(0,0),
    MANUAL(0,0);

    private @Getter Angle angle;
    private @Getter Voltage rollerVoltage;

    /**
     * Goal has angle, and rollerVoltage associated.
     *
     * @param angle
     * @param rollerVoltage
     */

    private Goal(double angle, double rollerVoltage) {
      this.angle = Units.Radians.of(angle);
      this.rollerVoltage = Units.Volts.of(rollerVoltage);
    }

    /**
     * Goal has angle, and rollerVoltage associated.
     * @param goal The other goal to copy the angle and roller voltage of.
     * @param beltVoltage The goal belt voltage.
     */
    private Goal(Goal goal) {
      this(goal.getAngle().in(Units.Radians), goal.getRollerVoltage().in(Units.Volts));
    }

    /**
     * Goal has angle, and rollerVoltage associated.
     * @param rollerVoltage The goal rpller voltage.
     * @param goal The other goal to copy the angle and belt voltage of.
     */
    private Goal(double rollerVoltage, Goal goal) {
      this(goal.getAngle().in(Units.Radians), rollerVoltage);
    }
  }

  private enum CoralGamePieceState {
    IDLE,
    INTAKING,
    INTAKE_ADJUSTING,
    INTAKE_HOLDING
  }

  private CoralGamePieceState currentCoralGamePieceState;
  private @Getter Goal currentGoal = Goal.STOW;

  private final Photoelectric handoffSensor;
  private final Photoelectric centerSensor;

  private final PivotIO pivotIO;
  private final PivotInputsAutoLogged pivotInputs = new PivotInputsAutoLogged();
  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final BeltIO beltIO;
  private final BeltInputsAutoLogged beltInputs = new BeltInputsAutoLogged();

  private SysIdRoutine routine;

  /**
   * Initializes Coral Intake with IO classes
   * @param beltIO
   * @param pivotIO
   * @param rollerIO
   */
  public CoralIntake(BeltIO beltIO, PivotIO pivotIO, RollerIO rollerIO, Photoelectric centerSensor, Photoelectric handoffSensor) {
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.beltIO = beltIO;
    this.centerSensor = centerSensor;
    this.handoffSensor = handoffSensor;

    SysIdRoutine.Mechanism sysIdMech = new SysIdRoutine.Mechanism(
        pivotIO::setVoltage,
        this::motorSysIdLog,
        this
    );

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
    beltIO.updateInputs(beltInputs);
    Logger.processInputs(getName() + "/pivot", beltInputs);
    Logger.processInputs(getName() + "/roller", beltInputs);
    Logger.processInputs(getName() + "/belt", beltInputs);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());

    Voltage desiredBeltVoltage;
    Voltage desiredRollerVoltage = currentGoal.getRollerVoltage();

    // Goal switch case

    Angle desiredAngle = currentGoal.getAngle();

    switch (getCurrentGoal()) {
      case HANDOFF:
        pivotIO.setPosition(desiredAngle);
        rollerIO.setVoltage(desiredRollerVoltage);
        beltIO.setVoltage(desiredBeltVoltage);
        break;
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

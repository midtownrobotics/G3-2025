package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral_intake.belt.BeltIO;
import frc.robot.subsystems.coral_intake.belt.BeltInputsAutoLogged;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.coral_intake.roller.RollerIO;
import frc.robot.subsystems.coral_intake.roller.RollerInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.Constraint;

import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  public enum State {
    // TODO: find angle out of the way of the carriage
    STOW(0,0,0),
    GROUND_INTAKE(0,7,0),
    GROUND_VOMIT(0,-7,0),
    STATION_INTAKE(0, 0, 0),
    HANDOFF(0,0,7),
    REVERSE_HANDOFF(0,0,-7),
    // TODO: find angle out of the way of climbing probably almost all the way down
    CLIMB(0,0,0),
    TUNING(0,0,0),
    MANUAL(0,0,0);

    private @Getter Angle angle;
    private @Getter Voltage rollerVoltage;
    private @Getter Voltage beltVoltage;

    /**
     * State has angle, beltVoltage, and rollerVoltage associated.
     *
     * @param angle
     * @param rollerVoltage
     * @param beltVoltage
     */

    private State(double angle, double rollerVoltage, double beltVoltage) {
      this.angle = Angle.ofBaseUnits(angle, Radians);
      this.rollerVoltage = Voltage.ofBaseUnits(rollerVoltage, Volts);
      this.beltVoltage = Voltage.ofBaseUnits(beltVoltage, Volts);
    }
  }

  private @Getter @Setter State currentState = State.STOW;

  private final Distance handoffElevatorPosition = Distance.ofBaseUnits(0, Inches);
  private final Angle pivotOffset = Radians.of(0);

  // private Constraint<Angle> pivotConstraint = new Constraint<Angle>(Radians.of(0), Radians.of(0));

  private final BeltIO beltIO;
  private final BeltInputsAutoLogged beltInputs = new BeltInputsAutoLogged();
  private final PivotIO pivotIO;
  private final PivotInputsAutoLogged pivotInputs = new PivotInputsAutoLogged();
  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final Supplier<Distance> getElevatorPosition;

  /**
   * Initializes Coral Intake with IO classes
   * @param beltIO
   * @param pivotIO
   * @param rollerIO
   */
  public CoralIntake(BeltIO beltIO, PivotIO pivotIO, RollerIO rollerIO, Supplier<Distance> getElevatorPosition) {
    this.beltIO = beltIO;
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.getElevatorPosition = getElevatorPosition;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    beltIO.updateInputs(beltInputs);
    pivotIO.updateInputs(pivotInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(getName() + "/belt", beltInputs);
    Logger.processInputs(getName() + "/pivot", beltInputs);
    Logger.processInputs(getName() + "/roller", beltInputs);

    // state switch case

    Angle desiredAngle = currentState.getAngle();
    Voltage desiredBeltVoltage = currentState.getBeltVoltage();
    Voltage desiredRollerVoltage = currentState.getRollerVoltage();

    // desiredAngle = pivotConstraint.apply(desiredAngle, getPivotPosition());

    switch (currentState) {
      case HANDOFF:
        rollerIO.setVoltage(desiredRollerVoltage);
        pivotIO.setPosition(desiredAngle);
        if (getElevatorPosition.get().equals(handoffElevatorPosition)) {;
          break;
        }
        beltIO.setVoltage(desiredBeltVoltage);
        break;
      default:
        pivotIO.setPosition(desiredAngle);
        beltIO.setVoltage(desiredBeltVoltage);
        rollerIO.setVoltage(desiredRollerVoltage);
        break;
    }

    Logger.recordOutput(getName() + "/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  public Angle getPivotPosition() {
    return pivotInputs.absolutePosition.plus(pivotOffset);
  }
}

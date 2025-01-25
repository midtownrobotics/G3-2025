package frc.robot.subsystems.coral_intake;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.coral_intake.belt.BeltIO;
import frc.robot.subsystems.coral_intake.belt.BeltInputsAutoLogged;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.coral_intake.roller.RollerIO;
import frc.robot.subsystems.coral_intake.roller.RollerInputsAutoLogged;
import frc.robot.subsystems.superstructure.Superstructure;
import lombok.Getter;
import lombok.Setter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  public enum State {
    STOW(0,0,0),
    // TODO: find angle out of the way of the carriage
    GROUND_INTAKE(0,0,7),
    GROUND_VOMIT(0,0,-7),
    HANDOFF(0,7,0),
    REVERSE_HANDOFF(0,-7,0),
    CLIMB(0,0,0),
    // TODO: find angle out of the way of climbing
    TUNING(0,0,0),
    MANUAL(0,0,0);

    private @Getter Angle angle;
    private @Getter Voltage beltVoltage;
    private @Getter Voltage rollerVoltage; 

    /**
     * State has angle, beltVoltage, and rollerVoltage associated.
     *
     * @param angle 
     * @param beltVoltage
     * @param rollerVoltage
     */

    private State(double angle, int beltVoltage, int rollerVoltage) {
      this.angle = Angle.ofBaseUnits(angle, Radians);
      this.beltVoltage = Voltage.ofBaseUnits(beltVoltage, Volts);
      this.rollerVoltage = Voltage.ofBaseUnits(rollerVoltage, Volts);
    }
  }

  private @Getter @Setter State currentState = State.STOW;

  private final Distance handoffElevatorPosition = Distance.ofBaseUnits(0, Inches);
  private final Voltage handoffCoralOuttakeVoltage = Voltage.ofBaseUnits(0, Volts);


  private final BeltIO beltIO;
  private final BeltInputsAutoLogged beltInputs = new BeltInputsAutoLogged();
  private final PivotIO pivotIO;
  private final PivotInputsAutoLogged pivotInputs = new PivotInputsAutoLogged();
  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final Supplier<Distance> getElevatorPosition;
  private final Supplier<Voltage> getCoralOuttakeVoltage;

  /**
   * Initializes Coral Intake with IO classes
   * @param beltIO
   * @param pivotIO
   * @param rollerIO
   */
  public CoralIntake(BeltIO beltIO, PivotIO pivotIO, RollerIO rollerIO, Supplier<Distance> getElevatorPosition, Supplier<Voltage> getCoralOuttakeVoltage) {
    this.beltIO = beltIO;
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.getElevatorPosition = getElevatorPosition;
    this.getCoralOuttakeVoltage = getCoralOuttakeVoltage;
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
    Angle angle = currentState.getAngle();
    Voltage beltVoltage = currentState.getBeltVoltage();
    Voltage rollerVoltage = currentState.getRollerVoltage();

    switch (currentState) {
      case STOW:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case GROUND_INTAKE:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case GROUND_VOMIT:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case HANDOFF: 
        if (getElevatorPosition.get().equals(handoffElevatorPosition) | getCoralOuttakeVoltage.get().equals(handoffCoralOuttakeVoltage)) {;
          break;
        }
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case REVERSE_HANDOFF:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case CLIMB:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case TUNING:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case MANUAL:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
    }
  }

  public Angle getPivotPosition() {
    return null;
  }

  // setState()
  // getState()
}

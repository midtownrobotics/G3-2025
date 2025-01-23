package frc.robot.subsystems.coral_intake;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  public enum State {
    STOW(0,0,0),
    GROUND_INTAKE(0,0,0),
    GROUND_VOMIT(0,0,0),
    STATION_VOMIT(0,0,0),
    HANDOFF(0,0,0),
    REVERSE_HANDOFF(0,0,0),
    STATION_INTAKE(0,0,0),
    CLIMB(0,0,0),
    TUNING(0,0,0),
    MANUAL(0,0,0);

    private @Getter double angle;
    private @Getter int beltVoltage;
    private @Getter int rollerVoltage; 

    /**
     * State has angle, beltVoltage, and rollerVoltage associated.
     *
     * @param angle 
     * @param beltVoltage
     * @param rollerVoltage
     */

    private State(double angle, int beltVoltage, int rollerVoltage) {
      this.angle = angle;
      this.beltVoltage = beltVoltage;
      this.rollerVoltage = rollerVoltage;
    }
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
    Angle angle = Angle.ofBaseUnits(currentState.getAngle(), Radians);
    int beltVoltage = currentState.getBeltVoltage();
    int rollerVoltage = currentState.getRollerVoltage();

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
      case STATION_VOMIT:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case HANDOFF:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case REVERSE_HANDOFF:
        pivotIO.setPosition(angle);
        beltIO.setVoltage(beltVoltage);
        rollerIO.setVoltage(rollerVoltage);
      case STATION_INTAKE:
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

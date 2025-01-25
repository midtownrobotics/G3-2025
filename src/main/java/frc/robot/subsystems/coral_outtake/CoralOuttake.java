package frc.robot.subsystems.coral_outtake;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral_outtake.roller.RollerIO;
import frc.robot.subsystems.coral_outtake.roller.RollerInputsAutoLogged;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;


public class CoralOuttake extends SubsystemBase {

  private static final Distance ELEVATOR_HANDOFF_THRESHOLD = Units.Inches.of(0);

  public enum State {
    IDLE,
    SHOOT,
    REVERSE_HANDOFF,
    HANDOFF,
    STATION_INTAKE,
    TUNING,
    MANUAL
  }

  private @Getter @Setter State currentState = State.IDLE;
  private RollerIO rollerIO;
  private Supplier<Distance> getElevatorPosition;
  private RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();


  /**
   * Initializes Coral Outtake
   * @param rollerIO
   */
  public CoralOuttake(RollerIO rollerIO, Supplier<Distance> getElevatorPosition ) {
    this.rollerIO = rollerIO;
    this.getElevatorPosition = getElevatorPosition;
  }

public AngularVelocity getRollerSpeed() {
  return rollerInputs.velocity;
}

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);

    // state switch case
    switch (currentState) {
      case IDLE:
        rollerIO.setVoltage(Units.Volts.zero());
        break;
      case SHOOT:
        rollerIO.setVoltage( CoralOuttakeConstants.SHOOT_VOLTAGE);
        break;
      case REVERSE_HANDOFF:
        if (Math.abs(Elevator.State.HANDOFF.getHeight().minus(getElevatorPosition.get()).in(Units.Meters)) < ELEVATOR_HANDOFF_THRESHOLD.in(Units.Meters) ) {
          rollerIO.setVoltage( CoralOuttakeConstants.HANDOFF_VOLTAGE.times(-1));
        }
        break;
      case HANDOFF:
        if (Math.abs(Elevator.State.HANDOFF.getHeight().minus(getElevatorPosition.get()).in(Units.Meters)) < ELEVATOR_HANDOFF_THRESHOLD.in(Units.Meters) ) {
          rollerIO.setVoltage( CoralOuttakeConstants.HANDOFF_VOLTAGE);
        }
        break;
      case STATION_INTAKE:
        rollerIO.setVoltage( CoralOuttakeConstants.STATION_INTAKE_VOLTAGE);
        break;
      case TUNING:
        rollerIO.setVoltage(Units.Volts.zero());
        break;
      case MANUAL:
        rollerIO.setVoltage(Units.Volts.zero());
        break;
      default:
        break;
    }

    Logger.recordOutput(getName() + "/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }
}

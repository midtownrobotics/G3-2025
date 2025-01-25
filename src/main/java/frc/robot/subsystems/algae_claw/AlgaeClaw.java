package frc.robot.subsystems.algae_claw;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.Constraints;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.algae_claw.roller.RollerIO;
import frc.robot.subsystems.algae_claw.roller.RollerInputsAutoLogged;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristInputsAutoLogged;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
  public enum State {
    STOW(0, 0),
    VOMIT(0, 0),
    START_POSITION(0, 0),
    GROUND_INTAKE(0, 0),
    GROUND_VOMIT(0, 0),
    REEF_INTAKE(0, 0),
    BARGE_SHOOT_FRONT(0, 0),
    BARGE_SHOOT_BACK(0, 0),
    BARGE_PREPARE_FRONT(0, 0),
    BARGE_PREPARE_BACK(0, 0),
    PROCESSOR_SHOOT(0, 0),
    PROCESSOR_PREPARE(0, 0),
    STACKED_ALGAE_INTAKE(0, 0),
    STACKED_ALGAE_VOMIT(0, 0),
    CLIMB(0, 0),
    TUNING(0, 0),
    MANUAL(0, 0);

    @Getter private final Angle angle;
    @Getter private final Voltage rollerVoltage;

    private State(double angle, double rollerVoltage) {
      this.angle = Units.Radians.of(angle);
      this.rollerVoltage = Units.Volts.of(rollerVoltage);
    }

  }

  private @Getter @Setter State currentState = State.STOW;

  private final RollerIO rollerIO;
  private final RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
  private final WristIO wristIO;
  private final WristInputsAutoLogged wristInputs = new WristInputsAutoLogged();
  private final DigitalInput sensor;
  private Constraints<Angle> wristConstraints = new Constraints<>(null, null);

  /** Constructor for algae claw. */
  public AlgaeClaw(RollerIO rollerIO, WristIO wristIO, DigitalInput sensor) {
    this.rollerIO = rollerIO;
    this.wristIO = wristIO;
    this.sensor = sensor;
  }

  @AutoLogOutput
  public Angle getPosition() {
    return wristInputs.position;
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();

    Logger.processInputs(getName() + "/roller", rollerInputs);
    Logger.processInputs(getName() + "/wrist", wristInputs);
    rollerIO.updateInputs(rollerInputs);
    wristIO.updateInputs(wristInputs);

    // state switch case
    switch (currentState) {
      case BARGE_PREPARE_BACK:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case BARGE_PREPARE_FRONT:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case BARGE_SHOOT_BACK:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case BARGE_SHOOT_FRONT:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case CLIMB:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case GROUND_INTAKE:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case GROUND_VOMIT:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case MANUAL:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case PROCESSOR_PREPARE:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case PROCESSOR_SHOOT:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case REEF_INTAKE:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case STACKED_ALGAE_INTAKE:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case STACKED_ALGAE_VOMIT:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case START_POSITION:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      default:
      case STOW:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case TUNING:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
      case VOMIT:
        rollerIO.setVoltage(currentState.getRollerVoltage());
        wristIO.setPosition(wristConstraints.clamp(currentState.getAngle()));
        break;
    }

    // record outputs
    Logger.recordOutput(getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  public boolean senseAlgae() {
    return sensor.get();
  }
}

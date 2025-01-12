package frc.robot.subsystems.coral_intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

public class CoralIntake extends SubsystemBase {
  public enum State {
    STOW,
    START_POSITION,
    GROUND_INTAKE,
    SOURCE_INTAKE,
    GROUND_VOMIT,
    SOURCE_VOMIT,
    HANDOFF,
    REVERSE_HANDOFF,
    STATION_INTAKE,
    CLIMB,
    TUNING,
    MANUAL
  }

  private @Getter @Setter State currentState = State.STOW;

  @Override
  public void periodic() {
    // state switch case
    super.periodic();
  }

  // setState()
  // getState()
}

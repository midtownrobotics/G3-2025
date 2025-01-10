package frc.robot.subsystems.coral_intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  public enum State {
    STOW,
    VOMIT,
    START_POSITION,
    INTAKING,
    HANDOFF,
    REVERSE_HANDOFF,
    STATION_INTAKE,
    CLIMB,
    TUNING,
    MANUAL
  }

  @Override
  public void periodic() {
    // state switch case
    super.periodic();
  }

  // setState()
  // getState()
}

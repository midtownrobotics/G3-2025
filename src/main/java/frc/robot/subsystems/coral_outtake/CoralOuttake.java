package frc.robot.subsystems.coral_outtake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralOuttake extends SubsystemBase {

  public enum State {
    IDLE,
    SHOOT,
    REVERSE_HANDOFF,
    HANDOFF,
    STATION_INTAKE,
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

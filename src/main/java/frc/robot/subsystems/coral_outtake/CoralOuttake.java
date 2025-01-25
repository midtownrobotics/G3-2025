package frc.robot.subsystems.coral_outtake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team1648.Limelight;
import frc.lib.team1648.RobotTime;
import frc.robot.subsystems.coral_outtake.roller.RollerIO;
import frc.robot.subsystems.coral_outtake.roller.RollerInputsAutoLogged;
import frc.robot.utils.OuttakeUtils;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class CoralOuttake extends SubsystemBase {

  private double angleFromDistance;
  private double speedFromDistance;


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
  private final RollerIO rollerIO;
  private RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();

  public Outtake(Limelight limelight) {
    this.limelight = limelight;
  }

  /**
   * Initializes Coral Outtake
   * @param rollerIO
   */
  public CoralOuttake(RollerIO rollerIO) {
    this.rollerIO = rollerIO;
  }

public void setState(State to) {
  currentState = to;
}

public State getState() {
  return currentState;
}

public double getAngleFromDistance() {
  return OuttakeUtils.instance.getAngleFromDistance(limelight.getDistance());
}

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);

    // state switch case
    switch ( ) {
      case value:
        
        break;
    
      default:
        break;
    }

    Logger.recordOutput(getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }
}

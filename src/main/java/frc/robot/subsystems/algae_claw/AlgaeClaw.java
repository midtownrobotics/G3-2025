package frc.robot.subsystems.algae_claw;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {
  public LinearConstraint<AngleUnit, Angle> wristConstraint = new LinearConstraint<AngleUnit,Angle>(AlgaeClawConstants.algaeClawMinAngle, AlgaeClawConstants.algaeClawMaxAngle);

  @RequiredArgsConstructor
  public enum Goal {
    STOW(Degrees.of(90)),
    OUT(Degrees.of(0)),
    TUNING(Degrees.of(0)),
    MANUAL(Degrees.of(0));

    @Getter private final Angle angle;
  }

  private @Getter Goal currentGoal = Goal.STOW;

  private final WristIO wristIO;
  private final WristInputsAutoLogged wristInputs = new WristInputsAutoLogged();

  /** Constructor for algae claw. */
  public AlgaeClaw(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @AutoLogOutput
  public Angle getPosition() {
    return wristInputs.position;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    Logger.processInputs(getName() + "/wrist", wristInputs);
    wristIO.updateInputs(wristInputs);

    if (getCurrentGoal() != Goal.TUNING) {
      wristIO.setPosition(wristConstraint.getClampedValue(currentGoal.getAngle()));
    }

    Logger.recordOutput("AlgaeClaw/currentState", getCurrentGoal());
    Logger.recordOutput("AlgaeClaw/goalPosition", getCurrentGoal().getAngle().in(Degrees));
    Logger.recordOutput("AlgaeClaw/anglePosition", wristInputs.position.in(Degrees));
    // record outputs
    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setGoal(Goal goal) {
    currentGoal = goal;
  }

  /** Set the constraints */
  public void setConstraints(LinearConstraint<AngleUnit, Angle> constraints) {
    wristConstraint = constraints;
  }

}

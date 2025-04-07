package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.subsystems.coral_outtake_pivot.CoralOuttakePivot;
import frc.robot.subsystems.coral_outtake_pivot.CoralOuttakePivotConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.LoggerUtil;

public class Superstructure extends SubsystemBase {

  private final CoralIntake coralIntake;
  private final CoralOuttakePivot coralOuttakePivot;
  private final Elevator elevator;

  private static final Angle kMaxCoralIntakeAngleElevatorUp = Degrees.of(82);
  private static final Angle kMaxCoralIntakeAngleElevatorUpThreshold = Degrees.of(5);

  /** Construct the robot supersctructure. */
  public Superstructure(CoralIntake coralIntake, Elevator elevator, CoralOuttakePivot coralOuttake) {
    this.coralIntake = coralIntake;
    this.coralOuttakePivot = coralOuttake;
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    LinearConstraint<DistanceUnit, Distance> elevatorConstraints = new LinearConstraint<DistanceUnit, Distance>(
        ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);
    LinearConstraint<AngleUnit, Angle> coralIntakeConstraints = new LinearConstraint<AngleUnit, Angle>(
        CoralIntakeConstants.coralIntakeMinAngle, CoralIntakeConstants.coralIntakeMaxAngle);
    LinearConstraint<AngleUnit, Angle> coralOuttakeConstraints = new LinearConstraint<AngleUnit, Angle>(
        CoralOuttakePivotConstants.coralOuttakeMinAngle, CoralOuttakePivotConstants.coralOuttakeMaxAngle);

    Angle coralIntakePosition = coralIntake.getPosition();
    Angle coralIntakeGoalPosition = coralIntake.getCurrentGoal().getAngle();
    Distance elevatorPosition = elevator.getPosition();
    Distance elevatorGoalPosition = elevator.getCurrentGoal().getHeight();
    Angle coralOuttakePosition = coralOuttakePivot.getPosition();

    // Prevents coral intake from intercepting elevator
    if (elevatorGoalPosition.gt(Inches.of(1.5)) || elevatorPosition.gt(Inches.of(1.5))) {
      coralIntakeConstraints.setUpper(kMaxCoralIntakeAngleElevatorUp);

      // Prevents elevator from intercepting intake
      if (coralIntakePosition.gt(kMaxCoralIntakeAngleElevatorUp.plus(kMaxCoralIntakeAngleElevatorUpThreshold))) {
        elevatorConstraints.restrictToValue(elevatorPosition);
      }
    }

    if (elevatorPosition.lt(Inches.of(12))) {
      if (
        (
          coralIntakePosition.gt(Degrees.of(94))
          || coralIntakeGoalPosition.gt(Degrees.of(94))
        )
        && !(coralIntakePosition.gt(Degrees.of(124)))
      ) {
        coralOuttakeConstraints.setUpper(Degrees.of(-35));
      }

      if (coralIntakePosition.gt(Degrees.of(94)) && coralIntakeGoalPosition.lt(Degrees.of(94))) {
        coralOuttakeConstraints.setUpper(Degrees.of(-35));
      }

      if (coralOuttakePosition.gt(Degrees.of(-30).plus(Degrees.of(2)))) {
        if (coralIntakePosition.gt(Degrees.of(124))) {
          coralIntakeConstraints.setLower(Degrees.of(133));
        } else {
          coralIntakeConstraints.setUpper(Degrees.of(110));
        }
      }
    }

    coralIntake.setConstraints(coralIntakeConstraints);
    elevator.setConstraints(elevatorConstraints);
    coralOuttakePivot.setConstraints(coralOuttakeConstraints);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }
}

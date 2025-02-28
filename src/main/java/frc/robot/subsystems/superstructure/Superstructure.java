package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.LoggerUtil;

public class Superstructure extends SubsystemBase {

  private final CoralIntake coralIntake;
  private final CoralOuttake coralOuttake;
  private final Elevator elevator;

  private static final Angle kMaxCoralIntakeAngleElevatorUp = Degrees.of(88);
  private static final Angle kMaxCoralIntakeAngleElevatorUpThreshold = Degrees.of(5);



  /** Construct the robot supersctructure. */
  public Superstructure(CoralIntake coralIntake, Elevator elevator, CoralOuttake coralOuttake) {
    this.coralIntake = coralIntake;
    this.coralOuttake = coralOuttake;
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    LinearConstraint<DistanceUnit, Distance> elevatorConstraints = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);
    LinearConstraint<AngleUnit, Angle> coralIntakeConstraints = new LinearConstraint<AngleUnit,Angle>(CoralIntakeConstants.coralIntakeMinAngle, CoralIntakeConstants.coralIntakeMaxAngle);

    Angle coralIntakePosition = coralIntake.getPosition();
    Angle coralIntakeGoalPosition = coralIntake.getCurrentGoal().getAngle();
    Distance elevatorPosition = elevator.getPosition();
    Distance elevatorGoalPosition = elevator.getCurrentGoal().getHeight();

    if (elevatorGoalPosition.gt(Inches.of(1.5)) || elevatorPosition.gt(Inches.of(1.5))) {
      coralIntakeConstraints.setUpper(kMaxCoralIntakeAngleElevatorUp);

      if (coralIntakePosition.gt(kMaxCoralIntakeAngleElevatorUp.plus(kMaxCoralIntakeAngleElevatorUpThreshold))) {
        elevatorConstraints.restrictToValue(elevatorPosition);
      }
    }

    coralIntake.setConstraints(coralIntakeConstraints);
    elevator.setConstraints(elevatorConstraints);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }
}

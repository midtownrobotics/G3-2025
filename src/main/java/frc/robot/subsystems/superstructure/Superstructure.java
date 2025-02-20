package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.AlgaeMode;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggerUtil;

import static edu.wpi.first.units.Units.Degrees;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private ControllerPrioritySubset controllerPrioritySubset = new ControllerPrioritySubset();

  public CoralIntake coralIntake;

  private AlgaeMode algaeMode = AlgaeMode.PROCESSOR;

  /** Enables priority value on schedule. */
  public void enable(Priority priority) {
    controllerPrioritySubset.enable(priority);
  }

  /** Disables priority value on schedule. */
  public void disable(Priority priority) {
    controllerPrioritySubset.disable(priority);
  }

  /**
   * Command to enable a specific priority
   */
  public Command enablePriorityCommand(Priority priority) {
    return Commands.runOnce(() -> enable(priority));
  }

  /**
   * Command to disable a specific priority
   */
  public Command disablePriorityCommand(Priority priority) {
    return Commands.runOnce(() -> disable(priority));
  }

  /** Construct the robot supersctructure. */
  public Superstructure(CoralIntake coralIntake) {
    this.coralIntake = coralIntake;
  }
  private Map<Priority, CoralIntake.Goal> priorityToCoralIntakeGoal = Map.ofEntries(
    Map.entry(Priority.GROUND_INTAKE_CORAL, CoralIntake.Goal.GROUND_INTAKE),
    Map.entry(Priority.GROUND_VOMIT_CORAL, CoralIntake.Goal.GROUND_VOMIT),
    Map.entry(Priority.STATION_INTAKE_CORAL, CoralIntake.Goal.STATION_INTAKE)
  );

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    Set<CoralIntake.Goal> possibleCoralIntakeGoals = new HashSet<>(EnumSet.allOf(CoralIntake.Goal.class));

    // TODO: Work with somebody to figure out restrictions for each state
    LinearConstraint<AngleUnit, Angle> coralIntakeConstraint = new LinearConstraint<AngleUnit,Angle>(CoralIntakeConstants.coralIntakeMinAngle, CoralIntakeConstants.coralIntakeMaxAngle);

    if (Constants.tuningMode.get()) {
      possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.TUNING);
    } else {
      for (Priority priority : controllerPrioritySubset.getCurrentlyEnabled()) {
        switch (priority) {
          case PANIC:
            break;
          case MANUAL:
            break;
          case CLIMB:
            possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.CLIMB);
            break;
          case GROUND_INTAKE_CORAL:
          case GROUND_VOMIT_CORAL:
          case STATION_INTAKE_CORAL:
            if (!possibleCoralIntakeGoals.contains(priorityToCoralIntakeGoal.get(priority))) continue;

            possibleCoralIntakeGoals = Set.of(priorityToCoralIntakeGoal.get(priority));

            break;
          case GROUND_INTAKE_ALGAE:
          case GROUND_VOMIT_ALGAE:
          case STACKED_INTAKE_ALGAE:
          case STACKED_VOMIT_ALGAE:
            if (!isCoralIntakeOutside()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

              moveCoralIntakeOutside(possibleCoralIntakeGoals);
            }
            break;
          case PREPARE_SCORE_ALGAE:
            if (algaeMode == AlgaeMode.PROCESSOR) {
              if (!isCoralIntakeOutside()) {
                if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

                moveCoralIntakeOutside(possibleCoralIntakeGoals);
              }
            } else {
              if (!isCoralIntakeOutside()) {
                if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

                moveCoralIntakeOutside(possibleCoralIntakeGoals);
              }
            }
            break;
          case PREPARE_SCORE_CORAL:

            if (!isCoralIntakeOutside()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

              moveCoralIntakeOutside(possibleCoralIntakeGoals);
            }

            break;
          case SCORE_GAME_PIECE:
            break;
          case REVERSE_HANDOFF_CORAL:
          case HANDOFF_CORAL:

            if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

            moveCoralIntakeOutside(possibleCoralIntakeGoals);

            if (!possibleCoralIntakeGoals.contains(CoralIntake.Goal.HANDOFF)) continue;

            possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF);

            break;
        }
      }
    }

    // Coral intake handoff logic
 
    if (
      possibleCoralIntakeGoals.contains(CoralIntake.Goal.GROUND_INTAKE) 
      && coralIntake.isCoralDetected()
    ) {
      possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF);
    }

    if (possibleCoralIntakeGoals.contains(CoralIntake.Goal.HANDOFF)) {
      if (coralIntake.doesCoralNeedAdjusting()) {
        possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF_ADJUSTING);
      }
    }

    Angle coralIntakePosition = coralIntake.getPivotPosition();

    // TODO: Find positions for constraints

    // If coral intake would intersect elevator due to bad coral placement
    if (coralIntake.isCoralBlockingMovement()) {
      coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If elevator would intersect coral intake due to high coral intake
    if (coralIntakePosition.gte(Degrees.of(0))) {
       Logger.recordOutput("Superstructure/Constraints/ElevatorBlockedByIntake", coralIntakePosition.gte(Degrees.of(0)));
    }

    coralIntake.setGoal(possibleCoralIntakeGoals.iterator().next(), coralIntakeConstraint);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  } 


  private boolean isCoralIntakeOutside() {
    return coralIntake.getPivotPosition().lt(Units.Degrees.of(50));
  }

  private <S> boolean isAnyPossibleGoals(Set<S> currentlyPossible, Set<S> newlyImpossible) {
    if (currentlyPossible.size() > newlyImpossible.size()) return true;

    Set<S> cloneOfCurrentlyPossible = new HashSet<>(currentlyPossible);
    cloneOfCurrentlyPossible.removeAll(newlyImpossible);

    return cloneOfCurrentlyPossible.size() > 0;
  }

  private final Set<CoralIntake.Goal> insideCoralIntakeGoals = Set.of(CoralIntake.Goal.STOW, CoralIntake.Goal.HANDOFF);

  private boolean canMoveCoralIntakeOutside(Set<CoralIntake.Goal> possibleGoals) {
    return isAnyPossibleGoals(possibleGoals, insideCoralIntakeGoals);
  }
  private void moveCoralIntakeOutside(Set<CoralIntake.Goal> possibleGoals) {
    possibleGoals.removeAll(insideCoralIntakeGoals);
  }

}

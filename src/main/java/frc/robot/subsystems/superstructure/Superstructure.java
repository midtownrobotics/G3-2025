package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.AlgaeMode;
import frc.robot.controls.Controls;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggerUtil;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class Superstructure extends SubsystemBase {
  private ControllerPrioritySubset controllerPrioritySubset = new ControllerPrioritySubset();

  public AlgaeClaw algaeClaw;
  public CoralIntake coralIntake;
  public CoralOuttake coralOuttake;
  public Elevator elevator;
  public Controls controls;

  /** Construct the robot supersctructure. */
  public Superstructure(AlgaeClaw algaeClaw, CoralIntake coralIntake, CoralOuttake coralOuttake, Elevator elevator, Controls controls) {
    this.algaeClaw = algaeClaw;
    this.coralIntake = coralIntake;
    this.coralOuttake = coralOuttake;
    this.elevator = elevator;
    this.controls = controls;
  }

  /** Enables priority value on schedule. */
  public void enable(Priority priority) {
    controllerPrioritySubset.enable(priority);
  }

  /** Disables priority value on schedule. */
  public void disable(Priority priority) {
    controllerPrioritySubset.disable(priority);
  }

  private Map<Priority, CoralIntake.Goal> priorityToCoralIntakeGoal = Map.ofEntries(
    Map.entry(Priority.GROUND_INTAKE_CORAL, CoralIntake.Goal.GROUND_INTAKE),
    Map.entry(Priority.GROUND_VOMIT_CORAL, CoralIntake.Goal.GROUND_VOMIT),
    Map.entry(Priority.STATION_INTAKE_CORAL, CoralIntake.Goal.STATION_INTAKE)
  );

  private Map<Priority, AlgaeClaw.Goal> priorityToAlgaeClawGoal = Map.ofEntries(
    Map.entry(Priority.GROUND_INTAKE_ALGAE, AlgaeClaw.Goal.GROUND_INTAKE),
    Map.entry(Priority.GROUND_VOMIT_ALGAE, AlgaeClaw.Goal.GROUND_VOMIT),
    Map.entry(Priority.STACKED_INTAKE_ALGAE, AlgaeClaw.Goal.STACKED_ALGAE_INTAKE),
    Map.entry(Priority.STACKED_VOMIT_ALGAE, AlgaeClaw.Goal.STACKED_ALGAE_VOMIT)
  );

  private Map<Priority, Elevator.Goal> priorityToElevatorGoal = Map.ofEntries(
    Map.entry(Priority.GROUND_INTAKE_ALGAE, Elevator.Goal.ALGAE_GROUND),
    Map.entry(Priority.GROUND_VOMIT_ALGAE, Elevator.Goal.ALGAE_GROUND),
    Map.entry(Priority.STACKED_INTAKE_ALGAE, Elevator.Goal.ALGAE_STACKED),
    Map.entry(Priority.STACKED_VOMIT_ALGAE, Elevator.Goal.ALGAE_STACKED)
  );

  private Map<Priority, CoralOuttake.Goal> priorityToCoralOuttakeGoal = Map.ofEntries(
    Map.entry(Priority.REVERSE_HANDOFF_CORAL, CoralOuttake.Goal.REVERSE_HANDOFF),
    Map.entry(Priority.HANDOFF_CORAL, CoralOuttake.Goal.HANDOFF)
  );

  private Map<CoralMode, Elevator.Goal> coralModeToElevatorGoal = Map.ofEntries(
    Map.entry(CoralMode.L1, Elevator.Goal.L1),
    Map.entry(CoralMode.L2, Elevator.Goal.L2),
    Map.entry(CoralMode.L3, Elevator.Goal.L3),
    Map.entry(CoralMode.L4, Elevator.Goal.L4)
  );

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    Set<AlgaeClaw.Goal> possibleAlgaeClawGoals = new HashSet<>(EnumSet.allOf(AlgaeClaw.Goal.class));
    Set<CoralIntake.Goal> possibleCoralIntakeGoals = new HashSet<>(EnumSet.allOf(CoralIntake.Goal.class));
    Set<CoralOuttake.Goal> possibleCoralOuttakeGoals = new HashSet<>(EnumSet.allOf(CoralOuttake.Goal.class));
    Set<Elevator.Goal> possibleElevatorGoals = new HashSet<>(EnumSet.allOf(Elevator.Goal.class));

    // TODO: Work with somebody to figure out restrictions for each state
    LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);
    LinearConstraint<AngleUnit, Angle> coralIntakeConstraint = new LinearConstraint<AngleUnit,Angle>(CoralIntakeConstants.coralIntakeMinAngle, CoralIntakeConstants.coralIntakeMaxAngle);
    CircularConstraint algaeClawConstraint = new CircularConstraint();

    if (Constants.tuningMode.get()) {
      possibleAlgaeClawGoals = Set.of(AlgaeClaw.Goal.TUNING);
      possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.TUNING);
      possibleCoralOuttakeGoals = Set.of(CoralOuttake.Goal.TUNING);
      possibleElevatorGoals = Set.of(Elevator.Goal.TUNING);
    } else {
      for (Priority priority : controllerPrioritySubset.getCurrentlyEnabled()) {
        switch (priority) {
          case PANIC:
            break;
          case MANUAL:
            break;
          case CLIMB:
            possibleElevatorGoals = Set.of(Elevator.Goal.CLIMB);
            possibleAlgaeClawGoals = Set.of(AlgaeClaw.Goal.CLIMB);
            possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.CLIMB);
            break;
          case GROUND_INTAKE_CORAL:
          case GROUND_VOMIT_CORAL:
          case STATION_INTAKE_CORAL:
            if (!possibleCoralIntakeGoals.contains(priorityToCoralIntakeGoal.get(priority))) continue;

            if (!isCoralIntakeOutside() && isAlgaeClawBlockingIntake()) {
              if (!canMoveAlgaeClawOutside(possibleAlgaeClawGoals)) continue;
              moveAlgaeClawOutside(possibleAlgaeClawGoals);
            }

            possibleCoralIntakeGoals = Set.of(priorityToCoralIntakeGoal.get(priority));

            break;
          case GROUND_INTAKE_ALGAE:
          case GROUND_VOMIT_ALGAE:
          case STACKED_INTAKE_ALGAE:
          case STACKED_VOMIT_ALGAE:

            if (!possibleAlgaeClawGoals.contains(priorityToAlgaeClawGoal.get(priority)) || !possibleElevatorGoals.contains(priorityToElevatorGoal.get(priority))) continue;

            if (!isCoralIntakeOutside()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

              moveCoralIntakeOutside(possibleCoralIntakeGoals);
            }

            possibleElevatorGoals = Set.of(priorityToElevatorGoal.get(priority));
            possibleAlgaeClawGoals = Set.of(priorityToAlgaeClawGoal.get(priority));

            break;
          case PREPARE_SCORE_ALGAE:
            if (controls.getAlgaeMode() == AlgaeMode.PROCESSOR) {
              if (!possibleAlgaeClawGoals.contains(AlgaeClaw.Goal.PROCESSOR_PREPARE) || !possibleElevatorGoals.contains(Elevator.Goal.PROCESSOR)) continue;

              if (!isCoralIntakeOutside()) {
                if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

                moveCoralIntakeOutside(possibleCoralIntakeGoals);
              }

              possibleElevatorGoals = Set.of(Elevator.Goal.PROCESSOR);
              possibleAlgaeClawGoals = Set.of(AlgaeClaw.Goal.PROCESSOR_PREPARE);
            } else {
              if (!possibleAlgaeClawGoals.contains(AlgaeClaw.Goal.BARGE_PREPARE_BACK) || !possibleElevatorGoals.contains(Elevator.Goal.BARGE)) continue;

              if (!isCoralIntakeOutside()) {
                if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;
                if (!canMoveAlgaeClawOutside(possibleAlgaeClawGoals)) continue;

                moveCoralIntakeOutside(possibleCoralIntakeGoals);
                moveAlgaeClawOutside(possibleAlgaeClawGoals);
              } else {
                possibleAlgaeClawGoals = Set.of(AlgaeClaw.Goal.BARGE_PREPARE_BACK);
              }

              possibleElevatorGoals = Set.of(Elevator.Goal.BARGE);
            }
            break;
          case PREPARE_SCORE_CORAL:
            if (!possibleElevatorGoals.contains(coralModeToElevatorGoal.get(controls.getCoralMode()))) continue;

            if (!isCoralIntakeOutside()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;
              if (!canMoveAlgaeClawOutside(possibleAlgaeClawGoals)) continue;

              moveCoralIntakeOutside(possibleCoralIntakeGoals);
              moveAlgaeClawOutside(possibleAlgaeClawGoals);
            }

            possibleElevatorGoals = Set.of(coralModeToElevatorGoal.get(controls.getCoralMode()));
            possibleCoralOuttakeGoals = Set.of(CoralOuttake.Goal.SHOOT);

            break;
          case SCORE_GAME_PIECE:
            if (controllerPrioritySubset.getCurrentlyEnabled().contains(Priority.PREPARE_SCORE_ALGAE)) {
              if (!possibleCoralOuttakeGoals.contains(CoralOuttake.Goal.SHOOT)) continue;
              possibleCoralOuttakeGoals = Set.of(CoralOuttake.Goal.SHOOT);
            } else {
              if (!possibleAlgaeClawGoals.contains(AlgaeClaw.Goal.BARGE_PREPARE_BACK)) continue;
              possibleAlgaeClawGoals = Set.of(AlgaeClaw.Goal.BARGE_PREPARE_BACK);
            }
            break;
          case REVERSE_HANDOFF_CORAL:
          case HANDOFF_CORAL:

            if (!possibleCoralOuttakeGoals.contains(priorityToCoralOuttakeGoal.get(priority)) || !possibleElevatorGoals.contains(Elevator.Goal.HANDOFF)) continue;

            if (isElevatorExtended()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

              if (!isCoralIntakeOutside()) {
                if (!canMoveAlgaeClawOutside(possibleAlgaeClawGoals)) continue;

                moveAlgaeClawOutside(possibleAlgaeClawGoals);
              }

              moveCoralIntakeOutside(possibleCoralIntakeGoals);

              possibleElevatorGoals = Set.of(Elevator.Goal.HANDOFF);
              possibleCoralOuttakeGoals = Set.of(priorityToCoralOuttakeGoal.get(priority));
              continue;
            }

            if (!possibleCoralIntakeGoals.contains(CoralIntake.Goal.HANDOFF)) continue;

            if (isCoralIntakeOutside()) {
              if (!canMoveAlgaeClawOutside(possibleAlgaeClawGoals)) continue;

              moveAlgaeClawOutside(possibleAlgaeClawGoals);
            }

            possibleElevatorGoals = Set.of(Elevator.Goal.HANDOFF);
            possibleCoralOuttakeGoals = Set.of(priorityToCoralOuttakeGoal.get(priority));
            possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF);

            break;
        }
      }
    }

    // Coral intake handoff logic

    if (
      possibleCoralIntakeGoals.contains(CoralIntake.Goal.GROUND_INTAKE) 
      && possibleElevatorGoals.contains(Elevator.Goal.HANDOFF)
      && coralIntake.isCoralDetected()
    ) {
      possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF);
    }

    if (possibleCoralIntakeGoals.contains(CoralIntake.Goal.HANDOFF) && possibleElevatorGoals.contains(Elevator.Goal.HANDOFF)) {
      if (coralIntake.doesCoralNeedAdjusting()) {
        possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF_ADJUSTING);
      }
    }

    Angle algaeClawPosition = algaeClaw.getPosition();
    Angle coralIntakePosition = coralIntake.getPivotPosition();
    Distance elevatorPosition = elevator.getPosition();

    // TODO: Find positions for consrains

    // If coral intake would intersect elevator due to bad coral placement
    if (coralIntake.isCoralBlockingMovement()) {
      coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If coral intake would intersect elevator due to low elevator
    if (elevatorPosition.lte(Inches.of(0))) {
      coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If elevator would intersect coral intake due to high coral intake
    if (coralIntakePosition.gte(Degrees.of(0))) {
      elevatorConstraint.addKeepOutConstraint(Inches.of(0), Inches.of(0));
    }

    // If elevator would push algae claw into frame
    if (algaeClawPosition.gte(Degrees.of(0))) {
      elevatorConstraint.addKeepOutConstraint(Inches.of(0), Inches.of(0));
    }

    // If algae claw would intersect frame due to low elevator
    if (elevatorPosition.lte(Inches.of(0))) {
      algaeClawConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If algae claw would intersect the coral intake due to low elevator and high intake
    if (coralIntakePosition.gte(Degrees.of(0)) && elevatorPosition.lte(Inches.of(0))) {
      algaeClawConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If coral inatke would intersect algae claw due to low elevator
    if (algaeClawPosition.gte(Degrees.of(0)) && elevatorPosition.lte(Inches.of(0))) {
      coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If elevator would push algae claw into coral intake due to low algae claw and high intake
    if (coralIntakePosition.gte(Degrees.of(0)) && algaeClawPosition.gte(Degrees.of(0))) {
      elevatorConstraint.addKeepOutConstraint(Inches.of(0), Inches.of(0));
    }

    algaeClaw.setGoal(possibleAlgaeClawGoals.iterator().next(), algaeClawConstraint);
    coralIntake.setGoal(possibleCoralIntakeGoals.iterator().next(), coralIntakeConstraint);
    coralOuttake.setGoal(possibleCoralOuttakeGoals.iterator().next());
    elevator.setGoal(possibleElevatorGoals.iterator().next(), elevatorConstraint);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  } 

  private boolean isAlgaeClawBlockingIntake() {
    return (
      (
        algaeClaw.getPosition().gt(Units.Degrees.of(30)) ||
        algaeClaw.getPosition().lt(Units.Degrees.of(-90))
      ) &&
      elevator.getPosition().lt(Units.Inches.of(22))
    );
  }

  private boolean isElevatorExtended() {
    return elevator.getPosition().lt(Units.Inches.of(5));
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

  private final Set<AlgaeClaw.Goal> insideAlgaeClawGoals = Set.of(AlgaeClaw.Goal.BARGE_SHOOT_BACK, AlgaeClaw.Goal.BARGE_SHOOT_FRONT, AlgaeClaw.Goal.START_POSITION, AlgaeClaw.Goal.STOW);

  private boolean canMoveAlgaeClawOutside(Set<AlgaeClaw.Goal> possibleGoals) {
    return isAnyPossibleGoals(possibleGoals, insideAlgaeClawGoals);
  }
  private void moveAlgaeClawOutside(Set<AlgaeClaw.Goal> possibleGoals) {
    possibleGoals.removeAll(insideAlgaeClawGoals);
  }

  private final Set<CoralIntake.Goal> insideCoralIntakeGoals = Set.of(CoralIntake.Goal.STOW, CoralIntake.Goal.HANDOFF);

  private boolean canMoveCoralIntakeOutside(Set<CoralIntake.Goal> possibleGoals) {
    return isAnyPossibleGoals(possibleGoals, insideCoralIntakeGoals);
  }
  private void moveCoralIntakeOutside(Set<CoralIntake.Goal> possibleGoals) {
    possibleGoals.removeAll(insideCoralIntakeGoals);
  }

}

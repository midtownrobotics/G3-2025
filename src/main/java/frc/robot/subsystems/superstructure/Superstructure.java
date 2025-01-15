package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.Controls;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.elevator.Elevator;
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

  private Map<Priority, CoralIntake.State> priorityToCoralIntakeState = Map.ofEntries(
    Map.entry(Priority.GROUND_INTAKE_CORAL, CoralIntake.State.GROUND_INTAKE),
    Map.entry(Priority.GROUND_VOMIT_CORAL, CoralIntake.State.GROUND_VOMIT),
    Map.entry(Priority.STATION_INTAKE_CORAL, CoralIntake.State.STATION_INTAKE),
    Map.entry(Priority.STATION_VOMIT_CORAL, CoralIntake.State.STATION_VOMIT)
  );

  private Map<Priority, AlgaeClaw.State> priorityToAlgaeClawState = Map.ofEntries(
    Map.entry(Priority.GROUND_INTAKE_ALGAE, AlgaeClaw.State.GROUND_INTAKE),
    Map.entry(Priority.GROUND_VOMIT_ALGAE, AlgaeClaw.State.GROUND_VOMIT),
    Map.entry(Priority.STACKED_INTAKE_ALGAE, AlgaeClaw.State.STACKED_ALGAE_INTAKE),
    Map.entry(Priority.STACKED_VOMIT_ALGAE, AlgaeClaw.State.STACKED_ALGAE_VOMIT)
  );

  private Map<Priority, Elevator.State> priorityToElevatorState = Map.ofEntries(
    Map.entry(Priority.GROUND_INTAKE_ALGAE, Elevator.State.ALGAE_GROUND),
    Map.entry(Priority.GROUND_VOMIT_ALGAE, Elevator.State.ALGAE_GROUND),
    Map.entry(Priority.STACKED_INTAKE_ALGAE, Elevator.State.ALGAE_STACKED),
    Map.entry(Priority.STACKED_VOMIT_ALGAE, Elevator.State.ALGAE_STACKED)
  );

  private Map<CoralMode, Elevator.State> coralModeToElevatorState = Map.ofEntries(
    Map.entry(CoralMode.L1, Elevator.State.L1),
    Map.entry(CoralMode.L2, Elevator.State.L2),
    Map.entry(CoralMode.L3, Elevator.State.L3),
    Map.entry(CoralMode.L4, Elevator.State.L4)
  );

  @Override
  public void periodic() {
    Set<AlgaeClaw.State> possibleAlgaeClawStates = new HashSet<>(EnumSet.allOf(AlgaeClaw.State.class));
    Set<CoralIntake.State> possibleCoralIntakeStates = new HashSet<>(EnumSet.allOf(CoralIntake.State.class));
    Set<CoralOuttake.State> possibleCoralOuttakeStates = new HashSet<>(EnumSet.allOf(CoralOuttake.State.class));
    Set<Elevator.State> possibleElevatorStates = new HashSet<>(EnumSet.allOf(Elevator.State.class));

    for (Priority priority : controllerPrioritySubset.getCurrentlyEnabled()) {
      switch (priority) {
        case PANIC:
          break;
        case MANUAL:
          break;
        case CLIMB:
          break;
        case GROUND_INTAKE_CORAL:
        case GROUND_VOMIT_CORAL:
        case STATION_INTAKE_CORAL:
        case STATION_VOMIT_CORAL:
          if (!possibleCoralIntakeStates.contains(priorityToCoralIntakeState.get(priority))) continue;

          if (!isCoralIntakeOutside() && isAlgaeClawBlockingIntake()) {
            if (!tryMoveAlgaeClawOutside(possibleAlgaeClawStates)) continue;
            moveAlgaeClawOutside(possibleAlgaeClawStates);
          }

          possibleCoralIntakeStates = Set.of(priorityToCoralIntakeState.get(priority));

          break;
        case GROUND_INTAKE_ALGAE:
        case GROUND_VOMIT_ALGAE:
        case STACKED_INTAKE_ALGAE:
        case STACKED_VOMIT_ALGAE:

          if (!possibleAlgaeClawStates.contains(priorityToAlgaeClawState.get(priority)) || !possibleElevatorStates.contains(priorityToElevatorState.get(priority))) continue;

          if (!isCoralIntakeOutside()) {
            if (!tryMoveCoralIntakeOutside(possibleCoralIntakeStates)) continue;

            moveCoralIntakeOutside(possibleCoralIntakeStates);
          }

          possibleElevatorStates = Set.of(priorityToElevatorState.get(priority));
          possibleAlgaeClawStates = Set.of(priorityToAlgaeClawState.get(priority));

          break;
        case PREPARE_SCORE_PROCESSOR:
          if (!possibleAlgaeClawStates.contains(AlgaeClaw.State.PROCESSOR_PREPARE) || !possibleElevatorStates.contains(Elevator.State.PROCESSOR)) continue;

          if (!isCoralIntakeOutside()) {
            if (!tryMoveCoralIntakeOutside(possibleCoralIntakeStates)) continue;

            moveCoralIntakeOutside(possibleCoralIntakeStates);
          }

          possibleElevatorStates = Set.of(Elevator.State.PROCESSOR);
          possibleAlgaeClawStates = Set.of(AlgaeClaw.State.PROCESSOR_PREPARE);

          break;
        case PREPARE_SCORE_BARGE:
          if (!possibleAlgaeClawStates.contains(AlgaeClaw.State.BARGE_PREPARE_BACK) || !possibleElevatorStates.contains(Elevator.State.BARGE)) continue;

          if (!isCoralIntakeOutside()) {
            if (!tryMoveCoralIntakeOutside(possibleCoralIntakeStates)) continue;
            if (!tryMoveAlgaeClawOutside(possibleAlgaeClawStates)) continue;

            moveCoralIntakeOutside(possibleCoralIntakeStates);
            moveAlgaeClawOutside(possibleAlgaeClawStates);
          } else {
            possibleAlgaeClawStates = Set.of(AlgaeClaw.State.BARGE_PREPARE_BACK);
          }

          possibleElevatorStates = Set.of(Elevator.State.BARGE);
          break;
        case PREPARE_SCORE_CORAL:
          if (!possibleElevatorStates.contains(coralModeToElevatorState.get(controls.getCoralMode()))) continue;

          if (!isCoralIntakeOutside()) {
            if (!tryMoveCoralIntakeOutside(possibleCoralIntakeStates)) continue;
            if (!tryMoveAlgaeClawOutside(possibleAlgaeClawStates)) continue;

            moveCoralIntakeOutside(possibleCoralIntakeStates);
            moveAlgaeClawOutside(possibleAlgaeClawStates);
          }

          possibleElevatorStates = Set.of(coralModeToElevatorState.get(controls.getCoralMode()));
          possibleCoralOuttakeStates = Set.of(CoralOuttake.State.SHOOT);

          break;
        case SCORE_GAME_PIECE:
          break;
        case HANDOFF_CORAL:

          if (!possibleCoralOuttakeStates.contains(CoralOuttake.State.HANDOFF) || !possibleElevatorStates.contains(Elevator.State.HANDOFF)) continue;

          if (isElevatorExtended()) {
            if (!tryMoveCoralIntakeOutside(possibleCoralIntakeStates)) continue;

            if (!isCoralIntakeOutside()) {
              if (!tryMoveAlgaeClawOutside(possibleAlgaeClawStates)) continue;

              moveAlgaeClawOutside(possibleAlgaeClawStates);
            }

            moveCoralIntakeOutside(possibleCoralIntakeStates);

            possibleElevatorStates = Set.of(Elevator.State.HANDOFF);
            possibleCoralOuttakeStates = Set.of(CoralOuttake.State.HANDOFF);
            continue;
          }

          if (!possibleCoralIntakeStates.contains(CoralIntake.State.HANDOFF)) continue;

          if (isCoralIntakeOutside()) {
            if (!tryMoveAlgaeClawOutside(possibleAlgaeClawStates)) continue;

            moveAlgaeClawOutside(possibleAlgaeClawStates);
          }

          possibleElevatorStates = Set.of(Elevator.State.HANDOFF);
          possibleCoralOuttakeStates = Set.of(CoralOuttake.State.HANDOFF);
          possibleCoralIntakeStates = Set.of(CoralIntake.State.HANDOFF);

          break;
      }
    }
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

  private <S> boolean isAnyPossibleStates(Set<S> currentlyPossible, Set<S> newlyImpossible) {
    if (currentlyPossible.size() > newlyImpossible.size()) return true;

    Set<S> cloneOfCurrentlyPossible = new HashSet<>(currentlyPossible);
    cloneOfCurrentlyPossible.removeAll(newlyImpossible);

    return cloneOfCurrentlyPossible.size() > 0;
  }

  private final Set<AlgaeClaw.State> insideAlgaeClawStates = Set.of(AlgaeClaw.State.BARGE_SHOOT_BACK, AlgaeClaw.State.BARGE_SHOOT_FRONT, AlgaeClaw.State.START_POSITION, AlgaeClaw.State.STOW);

  private boolean tryMoveAlgaeClawOutside(Set<AlgaeClaw.State> possibleStates) {
    return isAnyPossibleStates(possibleStates, insideAlgaeClawStates);
  }
  private void moveAlgaeClawOutside(Set<AlgaeClaw.State> possibleStates) {
    possibleStates.removeAll(insideAlgaeClawStates);
  }

  private final Set<CoralIntake.State> insideCoralIntakeStates = Set.of(CoralIntake.State.STOW, CoralIntake.State.HANDOFF, CoralIntake.State.REVERSE_HANDOFF);

  private boolean tryMoveCoralIntakeOutside(Set<CoralIntake.State> possibleStates) {
    return isAnyPossibleStates(possibleStates, insideCoralIntakeStates);
  }
  private void moveCoralIntakeOutside(Set<CoralIntake.State> possibleStates) {
    possibleStates.removeAll(insideCoralIntakeStates);
  }

}

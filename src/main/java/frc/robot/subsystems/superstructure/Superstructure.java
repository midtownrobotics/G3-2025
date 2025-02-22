package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.AlgaeMode;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggerUtil;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private ControllerPrioritySubset controllerPrioritySubset = new ControllerPrioritySubset();

  public AlgaeClaw algaeClaw;
  public CoralIntake coralIntake;
  public Elevator elevator;

  private CoralMode coralMode = CoralMode.L4;
  private AlgaeMode algaeMode = AlgaeMode.PROCESSOR;

  /** Construct the robot supersctructure. */
  public Superstructure(AlgaeClaw algaeClaw, CoralIntake coralIntake, Elevator elevator) {
    this.algaeClaw = algaeClaw;
    this.coralIntake = coralIntake;
    this.elevator = elevator;
  }

  public void setCoralMode(CoralMode coralMode) {
    this.coralMode = coralMode;
  }
  public void setAlgaeMode(AlgaeMode algaeMode) {
    this.algaeMode = algaeMode;
  }

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

  /**
   * Command to set a coral mode
   */
  public Command setCoralModeCommand(CoralMode coralMode) {
    return Commands.runOnce(() -> setCoralMode(coralMode));
  }

  /**
   * Command to increment the coral mode
   */
  public Command incrementCoralModeCommand() {
    return Commands.runOnce(() -> setCoralMode(coralMode.increment()));
  }

  /**
   * Command to decrement the coral mode
   */
  public Command decrementCoralModeCommand() {
    return Commands.runOnce(() -> setCoralMode(coralMode.decrement()));
  }

  /**
   * Command to set the algae mode
   */
  public Command setAlgaeModeCommand(AlgaeMode algaeMode) {
    return Commands.runOnce(() -> setAlgaeMode(algaeMode));
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
    Set<Elevator.Goal> possibleElevatorGoals = new HashSet<>(EnumSet.allOf(Elevator.Goal.class));

    // TODO: Work with somebody to figure out restrictions for each state
    LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);
    LinearConstraint<AngleUnit, Angle> coralIntakeConstraint = new LinearConstraint<AngleUnit,Angle>(CoralIntakeConstants.coralIntakeMinAngle, Radians.of(CoralIntakeConstants.coralIntakeMaxAngle.get()));
    CircularConstraint algaeClawConstraint = new CircularConstraint();

    if (Constants.tuningMode.get()) {
      possibleAlgaeClawGoals = Set.of(AlgaeClaw.Goal.TUNING);
      possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.TUNING);
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
            if (algaeMode == AlgaeMode.PROCESSOR) {
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
            if (!possibleElevatorGoals.contains(coralModeToElevatorGoal.get(coralMode))) continue;

            if (!isCoralIntakeOutside()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;
              if (!canMoveAlgaeClawOutside(possibleAlgaeClawGoals)) continue;

              moveCoralIntakeOutside(possibleCoralIntakeGoals);
              moveAlgaeClawOutside(possibleAlgaeClawGoals);
            }

            possibleElevatorGoals = Set.of(coralModeToElevatorGoal.get(coralMode));

            break;
          case SCORE_GAME_PIECE:
            if (controllerPrioritySubset.getCurrentlyEnabled().contains(Priority.PREPARE_SCORE_ALGAE)) {
            } else {
              if (!possibleAlgaeClawGoals.contains(AlgaeClaw.Goal.BARGE_PREPARE_BACK)) continue;
              possibleAlgaeClawGoals = Set.of(AlgaeClaw.Goal.BARGE_PREPARE_BACK);
            }
            break;
          case REVERSE_HANDOFF_CORAL:
          case HANDOFF_CORAL:
            if (isElevatorExtended()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

              if (!isCoralIntakeOutside()) {
                if (!canMoveAlgaeClawOutside(possibleAlgaeClawGoals)) continue;

                moveAlgaeClawOutside(possibleAlgaeClawGoals);
              }

              moveCoralIntakeOutside(possibleCoralIntakeGoals);

              possibleElevatorGoals = Set.of(Elevator.Goal.HANDOFF);
              continue;
            }

            if (!possibleCoralIntakeGoals.contains(CoralIntake.Goal.HANDOFF)) continue;

            if (isCoralIntakeOutside()) {
              if (!canMoveAlgaeClawOutside(possibleAlgaeClawGoals)) continue;

              moveAlgaeClawOutside(possibleAlgaeClawGoals);
            }

            possibleElevatorGoals = Set.of(Elevator.Goal.HANDOFF);
            possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF);

            break;
        }
      }
    }

    if (algaeClaw.hasAlgae()) {
      
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

    // TODO: Find positions for constraints

    // If coral intake would intersect elevator due to bad coral placement
    if (coralIntake.isCoralBlockingMovement()) {
      coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If coral intake would intersect elevator due to low elevator
    if (elevatorPosition.lte(Inches.of(0))) {
      Logger.recordOutput("Superstructure/Constraints/IntakeBlockedByElevator", elevatorPosition.lte(Inches.of(0)));
      coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If elevator would intersect coral intake due to high coral intake
    if (coralIntakePosition.gte(Degrees.of(0))) {
       Logger.recordOutput("Superstructure/Constraints/ElevatorBlockedByIntake", coralIntakePosition.gte(Degrees.of(0)));
      elevatorConstraint.addKeepOutConstraint(Inches.of(0), Inches.of(0));
    }

    // If elevator would push algae claw into frame
    if (algaeClawPosition.gte(Degrees.of(0))) {
       Logger.recordOutput("Superstructure/Constraints/ElevatorBlockedByClaw", algaeClawPosition.gte(Degrees.of(0)));
      elevatorConstraint.addKeepOutConstraint(Inches.of(0), Inches.of(0));
    }

    // If algae claw would intersect frame due to low elevator
    if (elevatorPosition.lte(Inches.of(0))) {
       Logger.recordOutput("Superstructure/Constraints/ClawBlockedByElevator", elevatorPosition.lte(Inches.of(0)));
      algaeClawConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If algae claw would intersect the coral intake due to low elevator and high intake
    if (coralIntakePosition.gte(Degrees.of(0)) && elevatorPosition.lte(Inches.of(0))) {
       Logger.recordOutput("Superstructure/Constraints/ClawBlockedByIntake", coralIntakePosition.gte(Degrees.of(0)) && elevatorPosition.lte(Inches.of(0)));
      algaeClawConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If coral intake would intersect algae claw due to low elevator
    if (algaeClawPosition.gte(Degrees.of(0)) && elevatorPosition.lte(Inches.of(0))) {
       Logger.recordOutput("Superstructure/Constraints/IntakeBlockedByClaw", algaeClawPosition.gte(Degrees.of(0)) && elevatorPosition.lte(Inches.of(0)));
      coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    }

    // If elevator would push algae claw into coral intake due to low algae claw and high intake
    if (coralIntakePosition.gte(Degrees.of(0)) && algaeClawPosition.gte(Degrees.of(0))) {
       Logger.recordOutput("Superstructure/Constraints/ElevatorBlockedByBoth", coralIntakePosition.gte(Degrees.of(0)) && algaeClawPosition.gte(Degrees.of(0)));
      elevatorConstraint.addKeepOutConstraint(Inches.of(0), Inches.of(0));
    }

    algaeClaw.setGoal(possibleAlgaeClawGoals.iterator().next(), algaeClawConstraint);
    coralIntake.setGoal(possibleCoralIntakeGoals.iterator().next(), coralIntakeConstraint);
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

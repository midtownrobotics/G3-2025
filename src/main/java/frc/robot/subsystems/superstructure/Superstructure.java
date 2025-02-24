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
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggerUtil;

import static edu.wpi.first.units.Units.Radians;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
  private ControllerPrioritySubset controllerPrioritySubset = new ControllerPrioritySubset();

  public CoralIntake coralIntake;
  public CoralOuttake coralOuttake;
  public Elevator elevator;

  @AutoLogOutput
  private CoralMode coralMode = CoralMode.L4;

  /** Construct the robot supersctructure. */
  public Superstructure(CoralIntake coralIntake, Elevator elevator, CoralOuttake coralOuttake) {
    this.coralIntake = coralIntake;
    this.coralOuttake = coralOuttake;
    this.elevator = elevator;

  }

  public void setCoralMode(CoralMode coralMode) {
    this.coralMode = coralMode;
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

  private Map<Priority, CoralIntake.Goal> priorityToCoralIntakeGoal = Map.ofEntries(
    Map.entry(Priority.GROUND_INTAKE_CORAL, CoralIntake.Goal.GROUND_INTAKE),
    Map.entry(Priority.GROUND_VOMIT_CORAL, CoralIntake.Goal.GROUND_VOMIT),
    Map.entry(Priority.STATION_INTAKE_CORAL, CoralIntake.Goal.STATION_INTAKE)
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

    Set<CoralIntake.Goal> possibleCoralIntakeGoals = EnumSet.allOf(CoralIntake.Goal.class);
    Set<CoralOuttake.Goal> possibleCoralOuttakeGoals = EnumSet.allOf(CoralOuttake.Goal.class);
    Set<Elevator.Goal> possibleElevatorGoals = EnumSet.allOf(Elevator.Goal.class);

    // TODO: Work with somebody to figure out restrictions for each state
    LinearConstraint<DistanceUnit, Distance> elevatorConstraint = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);
    LinearConstraint<AngleUnit, Angle> coralIntakeConstraint = new LinearConstraint<AngleUnit,Angle>(CoralIntakeConstants.coralIntakeMinAngle, Radians.of(CoralIntakeConstants.coralIntakeMaxAngle.get()));
    CircularConstraint algaeClawConstraint = new CircularConstraint();

    if (Constants.tuningMode.get()) {
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
            possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.CLIMB);
            break;
          case GROUND_INTAKE_CORAL:
          case GROUND_VOMIT_CORAL:
          case STATION_INTAKE_CORAL:
            if (!possibleCoralIntakeGoals.contains(priorityToCoralIntakeGoal.get(priority))) continue;

            possibleCoralIntakeGoals = Set.of(priorityToCoralIntakeGoal.get(priority));

            break;
          case PREPARE_SCORE_CORAL:
            if (!possibleElevatorGoals.contains(coralModeToElevatorGoal.get(coralMode))) continue;

            if (!isCoralIntakeOutside()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

              moveCoralIntakeOutside(possibleCoralIntakeGoals);
            }

            possibleElevatorGoals = Set.of(coralModeToElevatorGoal.get(coralMode));

            break;
          case SCORE_GAME_PIECE:
            possibleCoralOuttakeGoals = Set.of(CoralOuttake.Goal.SHOOT);
            break;
          case HANDOFF_CORAL:
            if (isElevatorExtended()) {
              if (!canMoveCoralIntakeOutside(possibleCoralIntakeGoals)) continue;

              moveCoralIntakeOutside(possibleCoralIntakeGoals);

              possibleElevatorGoals = Set.of(Elevator.Goal.HANDOFF);
              continue;
            }

            if (!possibleCoralIntakeGoals.contains(CoralIntake.Goal.HANDOFF)) continue;

            possibleElevatorGoals = Set.of(Elevator.Goal.HANDOFF);
            possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF);

            break;
        }
      }}

    // Coral intake handoff logic
 
    // if (
    //   possibleCoralIntakeGoals.contains(CoralIntake.Goal.GROUND_INTAKE) 
    //   && possibleElevatorGoals.contains(Elevator.Goal.HANDOFF)
    //   && coralIntake.isCoralDetected()
    // ) {
    //   possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF);
    // }

    // if (possibleCoralIntakeGoals.contains(CoralIntake.Goal.HANDOFF) && possibleElevatorGoals.contains(Elevator.Goal.HANDOFF)) {
    //   if (coralIntake.doesCoralNeedAdjusting()) {
    //     possibleCoralIntakeGoals = Set.of(CoralIntake.Goal.HANDOFF_ADJUSTING);
    //   }
    // }

    Angle coralIntakePosition = coralIntake.getPivotPosition();
    Distance elevatorPosition = elevator.getPosition();

    // TODO: Find positions for constraints

    // If coral intake would intersect elevator due to bad coral placement
    // if (coralIntake.isCoralBlockingMovement()) {
    //   coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    // }

    // If coral intake would intersect elevator due to low elevator
    // if (elevatorPosition.lte(Inches.of(0))) {
    //   Logger.recordOutput("Superstructure/Constraints/IntakeBlockedByElevator", elevatorPosition.lte(Inches.of(0)));
    //   coralIntakeConstraint.addKeepOutConstraint(Degrees.of(0), Degrees.of(0));
    // }

    // If elevator would intersect coral intake due to high coral intake
    // if (coralIntakePosition.gte(Degrees.of(0))) {
    //    Logger.recordOutput("Superstructure/Constraints/ElevatorBlockedByIntake", coralIntakePosition.gte(Degrees.of(0)));
    //   elevatorConstraint.addKeepOutConstraint(Inches.of(0), Inches.of(0));
    // }

    // If elevator would push algae claw into coral intake due to low algae claw and high intake
    // if (coralIntakePosition.gte(Degrees.of(0)) && algaeClawPosition.gte(Degrees.of(0))) {
    //    Logger.recordOutput("Superstructure/Constraints/ElevatorBlockedByBoth", coralIntakePosition.gte(Degrees.of(0)) && algaeClawPosition.gte(Degrees.of(0)));
    //   elevatorConstraint.addKeepOutConstraint(Inches.of(0), Inches.of(0));
    // }

    coralIntake.setGoal(possibleCoralIntakeGoals.iterator().next(), coralIntakeConstraint);
    elevator.setGoal(possibleElevatorGoals.iterator().next(), elevatorConstraint);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  
  } 

  private boolean isElevatorExtended() {
    return elevator.getPosition().gt(Units.Inches.of(5));
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

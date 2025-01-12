package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashSet;
import java.util.Set;


public class Superstructure extends SubsystemBase {
  private ControllerPrioritySubset controllerPrioritySubset = new ControllerPrioritySubset();

  public AlgaeClaw algaeClaw;
  public CoralIntake coralIntake;
  public CoralOuttake coralOuttake;
  public Elevator elevator;

  /** Construct the robot supersctructure. */
  public Superstructure(AlgaeClaw algaeClaw, CoralIntake coralIntake, CoralOuttake coralOuttake, Elevator elevator) {
    this.algaeClaw = algaeClaw;
    this.coralIntake = coralIntake;
    this.coralOuttake = coralOuttake;
    this.elevator = elevator;
  }

  /** Enables priority value on schedule. */
  public void enable(Priority priority) {
    controllerPrioritySubset.enable(priority);
  }

  /** Disables priority value on schedule. */
  public void disable(Priority priority) {
    controllerPrioritySubset.disable(priority);
  }

  @Override
  public void periodic() {
    Set<AlgaeClaw.State> impossibleAlgaeClawStates = new HashSet<>();
    Set<CoralIntake.State> impossibleCoralIntakeStates = new HashSet<>();
    Set<CoralOuttake.State> impossibleCoralOuttakeStates = new HashSet<>();
    Set<Elevator.State> impossibleElevatorStates = new HashSet<>();

    for (Priority priority : controllerPrioritySubset.getCurrentlyEnabled()) {
      switch (priority) {
        case PANIC:
          break;
        case MANUAL:
          break;
        case CLIMB:
          break;
        case GROUND_INTAKE_CORAL:
          coralIntake.setCurrentState(CoralIntake.State.GROUND_INTAKE);

          /**
           * If elevator is down, move claw if intake is inside.
           * If outside, it can extend freely.
           */

          break;
        case GROUND_VOMIT_CORAL:

          /**
           * If elevator is down, move claw if intake is inside.
           * If outside, it can extend freely.
           */

          break;
        case SOURCE_INTAKE_CORAL:

          /**
           * If elevator is down, move claw if intake is inside.
           * If outside, it can extend freely.
           */

          break;
        case SOURCE_VOMIT_CORAL:

          /**
           * If elevator is down, move claw if intake is inside.
           * If outside, it can extend freely.
           */

          break;
        case GROUND_INTAKE_ALGAE:

          /**
           * If intake is inside, it must be moved outside.
           * If intake is outside, its fine.
           */

          break;
        case GROUND_VOMIT_ALGAE:

          /**
           * If intake is inside, it must be moved outside.
           * If intake is outside, its fine.
           */

          break;
        case STACKED_INTAKE_ALGAE:

          /**
           * If intake is inside, it must be moved outside.
           * If intake is outside, its fine.
           */

          break;
        case STACKED_VOMIT_ALGAE:

          /**
           * If intake is inside, it must be moved outside.
           * If intake is outside, its fine.
           */

          break;
        case HANDOFF_CORAL:

          /**
           * If elevator is up, intake must move outside.
           * If intake is outside, algae must move outside, intake must move inside.
           * If intake is inside, already there.
           */

          break;
        case PREPARE_SCORE_ALGAE:

          /**
           * If intake is inside, algae claw must mvoe outside, intake must move outside.
           */

          break;
        case PREPARE_SCORE_CORAL:

          /**
           * If intake is inside, algae claw must mvoe outside, intake must move outside.
           */

          break;
        case SCORE_GAME_PIECE:
          break;
      }
    }
  }
}

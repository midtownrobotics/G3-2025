package frc.robot.subsystems.superstructure;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.elevator.Elevator;

public class Superstructure extends SubsystemBase {
    private ControllerPrioritySubset controllerPrioritySubset = new ControllerPrioritySubset();

    public AlgaeClaw algaeClaw;
    public CoralIntake coralIntake;
    public CoralOuttake coralOuttake;
    public Elevator elevator;

    public Superstructure() {
        algaeClaw = new AlgaeClaw();
        coralIntake = new CoralIntake();
        coralOuttake = new CoralOuttake();
        elevator = new Elevator();
    }

    public void enable(Priority priority) {
        controllerPrioritySubset.enable(priority);
    }

    public void disable(Priority priority) {
        controllerPrioritySubset.disable(priority);
    }

    @Override
    public void periodic() {
        Set<AlgaeClaw.State> possibleAlgaeClawStates = new HashSet<>(Arrays.asList(AlgaeClaw.State.values()));
        Set<CoralIntake.State> possibleCoralIntakeStates = new HashSet<>(Arrays.asList(CoralIntake.State.values()));
        Set<CoralOuttake.State> possibleCoralOuttakeStates = new HashSet<>(Arrays.asList(CoralOuttake.State.values()));
        Set<Elevator.State> possibleElevatorStates = new HashSet<>(Arrays.asList(Elevator.State.values()));

        for (Priority priority: controllerPrioritySubset.getCurrentlyEnabled()) {
            switch (priority) {
                case PANIC:
                    break;
                case MANUAL:
                    break;
                case CLIMB:
                    break;
                case GROUND_INTAKE_CORAL:
                    if (algaeClaw.)
                    break;
                case GROUND_VOMIT_CORAL:
                    break;
                case SOURCE_INTAKE_CORAL:
                    break;
                case SOURCE_VOMIT_CORAL:
                    break;
                case GROUND_INTAKE_ALGAE:
                    break;
                case GROUND_VOMIT_ALGAE:
                    break;
                case STACKED_INTAKE_ALGAE:
                    break;
                case STACKED_VOMIT_ALGAE:
                    break;
                case HANDOFF_CORAL:
                    break;
                case PREPARE_SCORE_ALGAE:
                    break;
                case PREPARE_SCORE_CORAL:
                    break;
                case SCORE_GAME_PIECE:
                    break;
            }
        }
    }
}

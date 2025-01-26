// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.Controls;
import frc.robot.controls.MatchXboxControls;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristIOKraken;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.coral_outtake.roller.RollerIOBag;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchIOKraken;
import frc.robot.subsystems.superstructure.Priority;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.RobotViz;
import lombok.Getter;

public class RobotContainer {

  private final Controls controls;

  private final Superstructure superstructure;

  @Getter private final AlgaeClaw algaeClaw;
  @Getter private final CoralIntake coralIntake;
  @Getter private final CoralOuttake coralOuttake;
  @Getter private final Elevator elevator;

  /** RobotContainer initialization */
  public RobotContainer() {
    controls = new MatchXboxControls(0, 1);
    configureBindings();

    WristIO wristIO = new WristIOKraken(0, 0);
    algaeClaw = new AlgaeClaw(null, wristIO);


    WinchIO winchIO = new WinchIOKraken(0, 0);
    elevator = new Elevator(winchIO);

    coralIntake = new CoralIntake(null, null, null, elevator::getPosition);

    RollerIOBag rollerIO = new RollerIOBag(0);

    coralOuttake = new CoralOuttake(rollerIO, elevator::getPosition);

    superstructure = new Superstructure(algaeClaw, coralIntake, coralOuttake, elevator, controls);

    new RobotViz(() -> {
      return null;
    }, () -> coralIntake.getPivotPosition(), () -> elevator.getPosition(), () -> algaeClaw.getPosition());
  }

  /** Configures bindings to oi */
  private void configureBindings() {

    // Driver

    controls
        .resetDriveHeading()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to reset the drive heading
                }));

    controls
        .driveBrake()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to activate drivetrain brakes ("X" Mode)
                }));

    controls
        .gamePieceLock()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to lock onto a game piece
                }));

    controls
        .leftPositionLock()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to lock onto the left position on the reef
                }));

    controls
        .rightPositionLock()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to lock onto the right position on the reef
                }));

    controls
        .reefAlgaePositionLock()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to lock onto the true right position for algae intake
                }));

    // Operator

    enableDisablePriorityControl(controls.groundIntakeCoral(), Priority.GROUND_INTAKE_CORAL);
    enableDisablePriorityControl(controls.groundVomitCoral(), Priority.GROUND_VOMIT_CORAL);
    enableDisablePriorityControl(controls.sourceIntakeCoral(), Priority.STATION_INTAKE_CORAL);

    enableDisablePriorityControl(controls.groundIntakeAlgae(), Priority.GROUND_INTAKE_ALGAE);
    enableDisablePriorityControl(controls.groundVomitAlgae(), Priority.GROUND_VOMIT_ALGAE);
    enableDisablePriorityControl(controls.stackedIntakeAlgae(), Priority.STACKED_INTAKE_ALGAE);
    enableDisablePriorityControl(controls.stackedVomitAlgae(), Priority.STACKED_VOMIT_ALGAE);

    enableDisablePriorityControl(controls.prepareScoreCoral(), Priority.PREPARE_SCORE_CORAL);
    enableDisablePriorityControl(controls.prepareScoreAlgae(), Priority.PREPARE_SCORE_ALGAE);

    enableDisablePriorityControl(controls.handoffCoral(), Priority.HANDOFF_CORAL);

    enableDisablePriorityControl(controls.scoreGamePiece(), Priority.SCORE_GAME_PIECE);

    enableDisablePriorityControl(controls.climb(), Priority.CLIMB);

    enableDisablePriorityControl(controls.panic(), Priority.PANIC);
    enableDisablePriorityControl(controls.climb(), Priority.MANUAL);

  }

  public void enableDisablePriorityControl(Trigger trigger, Priority priority) {
    trigger
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.enable(priority);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  superstructure.disable(priority);
                }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

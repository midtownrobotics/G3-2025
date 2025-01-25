// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.controls.Controls;
import frc.robot.controls.MatchXboxControls;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristIOKraken;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchIOKraken;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.RobotViz;
import lombok.Getter;

public class RobotContainer {

  private final Controls controls;

  @SuppressWarnings("unused")
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

    coralOuttake = new CoralOuttake(null);

    WinchIO winchIO = new WinchIOKraken(0, 0);
    elevator = new Elevator(winchIO);

    coralIntake = new CoralIntake(null, null, null, elevator::getPosition, coralOuttake::getRollerVoltage);

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

    controls
        .groundIntakeCoral()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to intake coral from the ground
                }));

    controls
        .groundVomitCoral()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to vomit coral from the ground
                }));

    controls
        .sourceIntakeCoral()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to intake coral from the source
                }));

    controls
        .sourceVomitCoral()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to vomit coral from the source
                }));

    // Operator Controls for Algae
    controls
        .groundIntakeAlgae()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to intake algae from the ground
                }));

    controls
        .groundVomitAlgae()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to vomit algae from the ground
                }));

    controls
        .stackedIntakeAlgae()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to intake stacked algae
                }));

    controls
        .stackedVomitAlgae()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to vomit stacked algae
                }));

    controls
        .prepareScoreAlgae()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to prepare for scoring algae
                }));

    controls
        .prepareScoreCoral()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to prepare for scoring coral
                }));

    controls
        .handoffCoral()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to hand off coral
                }));

    controls
        .scoreGamePiece()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to score a game piece
                }));

    controls
        .climb()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to activate the climbing mechanism
                }));

    controls
        .panic()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic for the emergency panic mode
                }));

    controls
        .setManualMode()
        .onTrue(
            new InstantCommand(
                () -> {
                  // Logic to switch the robot to manual mode
                }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

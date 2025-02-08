// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.Controls;
import frc.robot.controls.MatchXboxControls;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
import frc.robot.subsystems.algae_claw.roller.ACRollerIO;
import frc.robot.subsystems.algae_claw.roller.ACRollerIOKraken;
import frc.robot.subsystems.algae_claw.roller.ACRollerIOReplay;
import frc.robot.subsystems.algae_claw.roller.ACRollerIOSim;
import frc.robot.subsystems.algae_claw.wrist.WristIO;
import frc.robot.subsystems.algae_claw.wrist.WristIOKraken;
import frc.robot.subsystems.algae_claw.wrist.WristIOReplay;
import frc.robot.subsystems.algae_claw.wrist.WristIOSim;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.belt.BeltIO;
import frc.robot.subsystems.coral_intake.belt.BeltIONeo;
import frc.robot.subsystems.coral_intake.belt.BeltIOReplay;
import frc.robot.subsystems.coral_intake.belt.BeltIOSim;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotIONeo;
import frc.robot.subsystems.coral_intake.pivot.PivotIOReplay;
import frc.robot.subsystems.coral_intake.pivot.PivotIOSim;
import frc.robot.subsystems.coral_intake.roller.CIRollerIO;
import frc.robot.subsystems.coral_intake.roller.CIRollerIONeo;
import frc.robot.subsystems.coral_intake.roller.CIRollerIOReplay;
import frc.robot.subsystems.coral_intake.roller.CIRollerIOSim;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.coral_outtake.roller.CORollerIO;
import frc.robot.subsystems.coral_outtake.roller.CORollerIOBag;
import frc.robot.subsystems.coral_outtake.roller.CORollerIOReplay;
import frc.robot.subsystems.coral_outtake.roller.CORollerIOSim;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchIOKraken;
import frc.robot.subsystems.elevator.winch.WinchIOReplay;
import frc.robot.subsystems.elevator.winch.WinchIOSim;
import frc.robot.subsystems.superstructure.Priority;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotViz;
import lombok.Getter;

public class RobotContainer {

  private final Controls controls;

  private final Superstructure superstructure;

  @Getter private final AlgaeClaw algaeClaw;
  @Getter private final CoralIntake coralIntake;
  @Getter private final CoralOuttake coralOuttake;
  @Getter private final Elevator elevator;
  @Getter private final Drive drive;

  /** RobotContainer initialization */
  public RobotContainer() {
    controls = new MatchXboxControls(0, 1);
    configureBindings();

    // Algae Claw
    WristIO wristIO;
    ACRollerIO algaeClawRollerIO;

    // Elevator
    WinchIO winchIO;

    // Coral Intake
    BeltIO beltIO;
    PivotIO pivotIO;
    CIRollerIO coralIntakeRollerIO;

    // Coral Outtake
    CORollerIO rollerIO;

    // Drive
    GyroIO gyroIO;
    ModuleIO flModuleIO;
    ModuleIO frModuleIO;
    ModuleIO blModuleIO;
    ModuleIO brModuleIO;

    switch (Constants.MODE) {
        case REPLAY:
            // Algae Claw
            wristIO = new WristIOReplay();
            algaeClawRollerIO = new ACRollerIOReplay();

            // Elevator
            winchIO = new WinchIOReplay();

            // Coral Intake
            beltIO = new BeltIOReplay();
            pivotIO = new PivotIOReplay();
            coralIntakeRollerIO = new CIRollerIOReplay();

            // Coral Outtake
            rollerIO = new CORollerIOReplay();

            // Drive
            // TODO: Understand sim/replay
            gyroIO = new GyroIOPigeon2();
            flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft);
            frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight);
            blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft);
            brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight);
            break;
        case SIM:
            // Algae Claw
            wristIO = new WristIOSim();
            algaeClawRollerIO = new ACRollerIOSim();

            // Elevator
            winchIO = new WinchIOSim();

            // Coral Intake
            beltIO = new BeltIOSim();
            pivotIO = new PivotIOSim();
            coralIntakeRollerIO = new CIRollerIOSim();

            // Coral Outtake
            rollerIO = new CORollerIOSim();

            // Drive
            // TODO: Understand sim/replay
            gyroIO = new GyroIOPigeon2();
            flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft);
            frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight);
            blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft);
            brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight);
            break;
        default:
            // Algae Claw
            wristIO = new WristIOKraken(Ports.AlgaeClaw.WristMotor, Ports.AlgaeClaw.WristEncoder);
            algaeClawRollerIO = new ACRollerIOKraken(Ports.AlgaeClaw.AlgaeClawRoller);

            // Elevator
            winchIO = new WinchIOKraken(Ports.Elevator.WinchMotor, Ports.Elevator.WinchEncoder);

            // Coral Intake
            beltIO = new BeltIONeo(Ports.CoralIntake.Belt);
            pivotIO = new PivotIONeo(Ports.CoralIntake.PivotMotor, Ports.CoralIntake.PivotEncoder);
            coralIntakeRollerIO = new CIRollerIONeo(Ports.CoralIntake.CoralIntakeRoller);

            // Coral Outtake
            rollerIO = new CORollerIOBag(Ports.CoralOuttake.Roller);

            // Drive
            gyroIO = new GyroIOPigeon2();
            flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft);
            frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight);
            blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft);
            brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight);
            break;
    }

    algaeClaw = new AlgaeClaw(algaeClawRollerIO, wristIO);
    elevator = new Elevator(winchIO);
    coralIntake = new CoralIntake(beltIO, pivotIO, coralIntakeRollerIO);
    coralOuttake = new CoralOuttake(rollerIO);
    drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);

    superstructure = new Superstructure(algaeClaw, coralIntake, coralOuttake, elevator, controls);

    new RobotViz(() -> {
      return null;
    }, () -> coralIntake.getPivotPosition(), () -> elevator.getPosition(), () -> algaeClaw.getPosition());
  }

  /** Configures bindings to oi */
  private void configureBindings() {
    // Driver

    drive.runVelocity(new ChassisSpeeds(controls.getDriveLeft(), controls.getDriveForward(), controls.getDriveRotation()));

    controls
        .resetDriveHeading()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.resetDriveHeading();
                }));

    controls
        .driveBrake()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.stopWithX();
                }));

    controls
        .gamePieceLock()
        .onTrue(
            new InstantCommand(
                () -> {
                  // TODO: Logic to lock onto a game piece
                }));

    controls
        .leftPositionLock()
        .onTrue(
            new InstantCommand(
                () -> {
                  // TODO: Logic to lock onto the left position on the reef
                }));

    controls
        .rightPositionLock()
        .onTrue(
            new InstantCommand(
                () -> {
                  // TODO: Logic to lock onto the right position on the reef
                }));

    controls
        .reefAlgaePositionLock()
        .onTrue(
            new InstantCommand(
                () -> {
                  // TODO: Logic to lock onto the true right position for algae intake
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
    enableDisablePriorityControl(controls.setManualMode(), Priority.MANUAL);

    // TODO: Use Elevator and Wrist axes as well

    controls
        .outtakeShoot()
        .onTrue(
            new InstantCommand(
                () -> {
                    // TODO: Logic for outtake shoot action
                }));

    controls
        .algaeClawIntake()
        .onTrue(
            new InstantCommand(
                () -> {
                    // TODO: Logic for algae claw intake action
                }));

    controls
        .coralForward()
        .onTrue(
            new InstantCommand(
                () -> {
                    // TODO: Logic for coral forward action
                }));

    controls
        .coralBackward()
        .onTrue(
            new InstantCommand(
                () -> {
                    // TODO: Logic for coral backward action
                }));

    controls
        .coralIntakeRun()
        .onTrue(
            new InstantCommand(
                () -> {
                    //TODO:  Logic for coral intake run action
                }));

    controls
        .coralIntakeReverse()
        .onTrue(
            new InstantCommand(
                () -> {
                    // TODO: Logic for coral intake reverse action
                }));

  }

  /** Handles trigger by enablling priority onTrue and disabling onFalse. */
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

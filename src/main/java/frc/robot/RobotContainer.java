// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerIOBag;
import frc.lib.RollerIO.RollerIOKraken;
import frc.lib.RollerIO.RollerIONeo;
import frc.lib.RollerIO.RollerIOReplay;
import frc.lib.RollerIO.RollerIOSim;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.Controls;
import frc.robot.controls.MatchXboxControls;
import frc.robot.subsystems.algae_claw.AlgaeClaw;
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
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchIOKraken;
import frc.robot.subsystems.elevator.winch.WinchIOReplay;
import frc.robot.subsystems.elevator.winch.WinchIOSim;
import frc.robot.subsystems.superstructure.Priority;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotViz;
import lombok.Getter;

public class RobotContainer {

  private final Controls controls;

  private final Superstructure superstructure;

  SendableChooser<Command> m_autoChooser;

  @Getter private final AlgaeClaw algaeClaw;
  @Getter private final CoralIntake coralIntake;
  @Getter private final CoralOuttake coralOuttake;
  @Getter private final Elevator elevator;
  @Getter private final Drive drive;

  @Getter private CANBusStatusSignalRegistration elevatorCANBusHandler = new CANBusStatusSignalRegistration();
  @Getter private CANBusStatusSignalRegistration driveCANBusHandler = new CANBusStatusSignalRegistration();

  /** RobotContainer initialization */
  public RobotContainer() {
    // Algae Claw
    WristIO wristIO;
    RollerIO algaeClawRollerIO;

    // Elevator
    WinchIO winchIO;

    // Coral Intake
    BeltIO beltIO;
    PivotIO pivotIO;
    RollerIO coralIntakeRollerIO;

    // Coral Outtake
    RollerIO rollerIO;

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
            algaeClawRollerIO = new RollerIOReplay();

            // Elevator
            winchIO = new WinchIOReplay();

            // Coral Intake
            beltIO = new BeltIOReplay();
            pivotIO = new PivotIOReplay();
            coralIntakeRollerIO = new RollerIOReplay();

            // Coral Outtake
            rollerIO = new RollerIOReplay();

            // Drive
            gyroIO = new GyroIOPigeon2(driveCANBusHandler);
            flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft, driveCANBusHandler);
            frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight, driveCANBusHandler);
            blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft, driveCANBusHandler);
            brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight, driveCANBusHandler);
            break;
        case SIM:
            // Algae Claw
            wristIO = new WristIOSim();
            algaeClawRollerIO = new RollerIOSim();

            // Elevator
            winchIO = new WinchIOSim();

            // Coral Intake
            beltIO = new BeltIOSim();
            pivotIO = new PivotIOSim();
            coralIntakeRollerIO = new RollerIOSim();

            // Coral Outtake
            rollerIO = new RollerIOSim();

            // Drive
            gyroIO = new GyroIOPigeon2(driveCANBusHandler);
            flModuleIO = new ModuleIOSim(TunerConstants.FrontLeft);
            frModuleIO = new ModuleIOSim(TunerConstants.FrontRight);
            blModuleIO = new ModuleIOSim(TunerConstants.BackLeft);
            brModuleIO = new ModuleIOSim(TunerConstants.BackRight);
            break;
        default:
            // Algae Claw
            wristIO = new WristIOKraken(Ports.AlgaeClaw.WristMotor, Ports.AlgaeClaw.WristEncoder, elevatorCANBusHandler);
            algaeClawRollerIO = new RollerIOKraken(Ports.AlgaeClaw.AlgaeClawRoller, elevatorCANBusHandler);

            // Elevator
            winchIO = new WinchIOKraken(Ports.Elevator.WinchMotor, Ports.Elevator.WinchEncoder, elevatorCANBusHandler);

            // Coral Intake
            beltIO = new BeltIONeo(Ports.CoralIntake.Belt);
            pivotIO = new PivotIONeo(Ports.CoralIntake.PivotMotor, Ports.CoralIntake.PivotEncoder);
            coralIntakeRollerIO = new RollerIONeo(Ports.CoralIntake.CoralIntakeRoller);

            // Coral Outtake
            rollerIO = new RollerIOBag(Ports.CoralOuttake.Roller);

            // Drive
            gyroIO = new GyroIOPigeon2(driveCANBusHandler);
            flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft, driveCANBusHandler);
            frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight, driveCANBusHandler);
            blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft, driveCANBusHandler);
            brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight, driveCANBusHandler);
            break;
    }

    algaeClaw = new AlgaeClaw(algaeClawRollerIO, wristIO);
    elevator = new Elevator(winchIO);
    coralIntake = new CoralIntake(beltIO, pivotIO, coralIntakeRollerIO);
    coralOuttake = new CoralOuttake(rollerIO);
    drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);

    controls = new MatchXboxControls(0, 1);
    configureBindings();

    superstructure = new Superstructure(algaeClaw, coralIntake, coralOuttake, elevator, controls);

    new RobotViz(() -> {
      return null;
    }, () -> coralIntake.getPivotPosition(), () -> elevator.getPosition(), () -> algaeClaw.getPosition());

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /** Configures bindings to oi */
  private void configureBindings() {
    // Driver

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controls::getDriveForward, controls::getDriveLeft, controls::getDriveRotation));

    controls.resetDriveHeading().onTrue(drive.resetDriveHeadingCommand());

    controls.driveBrake().onTrue(drive.stopWithXCommand());

    controls.gamePieceLock().onTrue(Commands.none());

    controls.leftPositionLock().onTrue(Commands.none());

    controls.rightPositionLock().onTrue(Commands.none());

    controls.reefAlgaePositionLock().onTrue(Commands.none());

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

    controls.outtakeShoot().onTrue(Commands.none());

    controls.algaeClawIntake().onTrue(Commands.none());

    controls.coralForward().onTrue(Commands.none());

    controls.coralBackward().onTrue(Commands.none());

    controls.coralIntakeRun().onTrue(Commands.none());

    controls.coralIntakeReverse().onTrue(Commands.none());

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

  /** Returns the autonomous command */
  public Command getAutonomousCommand() {
    var selected = m_autoChooser.getSelected();

    if (selected == null) {
      return Commands.print("AHHHHHH");
    }

    return selected;
  }
}

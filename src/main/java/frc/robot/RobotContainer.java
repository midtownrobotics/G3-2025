// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerIOBag;
import frc.lib.RollerIO.RollerIOKraken;
import frc.lib.RollerIO.RollerIONeo;
import frc.lib.RollerIO.RollerIOReplay;
import frc.lib.RollerIO.RollerIOSim;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.Controls;
import frc.robot.controls.MatchXboxControls;
import frc.robot.sensors.Photoelectric;
import frc.robot.subsystems.coral_intake.CoralIntake;
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

  private Superstructure superstructure;

  SendableChooser<Command> m_autoChooser;

  @Getter private final CoralIntake coralIntake;
  @Getter private final CoralOuttake coralOuttake;
  @Getter private final Elevator elevator;
  @Getter private final Drive drive;

  @Getter private CANBusStatusSignalRegistration elevatorCANBusHandler = new CANBusStatusSignalRegistration();
  @Getter private CANBusStatusSignalRegistration driveCANBusHandler = new CANBusStatusSignalRegistration();

  /** RobotContainer initialization */
  public RobotContainer() {
    // Algae Claw
    RollerIO algaeClawRollerIO;

    // Elevator
    WinchIO winchIO;

    // Coral Intake
    RollerIO beltIO;
    PivotIO pivotIO;
    RollerIO coralIntakeRollerIO;
    Photoelectric centerSensor = new Photoelectric(Ports.CoralIntake.centerSensor);
    Photoelectric handoffSensor = new Photoelectric(Ports.CoralIntake.handoffSensor);

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
            algaeClawRollerIO = new RollerIOReplay();

            // Elevator
            winchIO = new WinchIOReplay();

            // Coral Intake
            beltIO = new RollerIOReplay();
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

            // Elevator
            winchIO = new WinchIOSim();

            // Coral Intake
            beltIO = new RollerIOSim();
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
            // wristIO = new WristIOKraken(Ports.AlgaeClaw.wristMotor, Ports.AlgaeClaw.wristEncoder, elevatorCANBusHandler);
            // algaeClawRollerIO = new RollerIOKraken(Ports.AlgaeClaw.algaeClawRoller, elevatorCANBusHandler);

            // Elevator
            winchIO = new WinchIOKraken(Ports.Elevator.LeftWinchMotor, 
                                        Ports.Elevator.RightWinchMotor, 
                                        Ports.Elevator.WinchEncoder,
                                        driveCANBusHandler);

            // Coral Intake
            beltIO = new RollerIONeo(Ports.CoralIntake.belt);
            pivotIO = new PivotIONeo(Ports.CoralIntake.pivotMotor, Ports.CoralIntake.pivotEncoder);
            coralIntakeRollerIO = new RollerIONeo(Ports.CoralIntake.coralIntakeRoller);

            // Coral Outtake
            rollerIO = new RollerIOBag(Ports.CoralOuttake.roller);

            // Drive
            gyroIO = new GyroIOPigeon2(driveCANBusHandler);
            flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft, driveCANBusHandler);
            frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight, driveCANBusHandler);
            blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft, driveCANBusHandler);
            brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight, driveCANBusHandler);
            break;
    }
                // Algae Claw
                algaeClawRollerIO = new RollerIOSim();

    elevator = new Elevator(winchIO);
    coralIntake = new CoralIntake(beltIO, pivotIO, coralIntakeRollerIO, centerSensor, handoffSensor);
    coralOuttake = new CoralOuttake(rollerIO);
    drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
    
    superstructure = new Superstructure(coralIntake, elevator, coralOuttake);

    controls = new MatchXboxControls(0, 1);
    configureBindings();


    new RobotViz(() -> {
      return null;
    }, () -> coralIntake.getPivotPosition(), () -> elevator.getPosition());

    NamedCommands.registerCommand("ScoreCoralLevel4", Commands.sequence(
      Commands.print("Raising Elevator..."),
      Commands.waitSeconds(1),
      Commands.print("Scoring..."),
      Commands.waitSeconds(0.5),
      Commands.print("Lowering Elevator..."),
      Commands.waitSeconds(0.3),
      Commands.print("Done.")));

    NamedCommands.registerCommand("IntakeFromLoadingStation", Commands.sequence(
      Commands.print("Intaking..."),
      Commands.waitSeconds(0.7),
      Commands.print("Done.")));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /** Configures bindings to oi */
  private void configureBindings() {
    // Driver

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controls::getDriveForward, controls::getDriveLeft, controls::getDriveRotation));

    controls.resetDriveHeading().onTrue(drive.resetDriveHeadingCommand());

    // controls.driveBrake().onTrue(drive.stopWithXCommand());

    // controls.gamePieceLock().onTrue(Commands.none());

    // controls.leftPositionLock().whileTrue(AutoBuilder.pathfindToPose(new Pose2d(5.27, 3.00, Rotation2d.fromDegrees(120)), new PathConstraints(FeetPerSecond.of(8), FeetPerSecondPerSecond.of(5), DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(480))));

    // controls.rightPositionLock().onTrue(Commands.none());

    // controls.reefAlgaePositionLock().onTrue(Commands.none());

    // controls.algaeModeBarge().onTrue(superstructure.setAlgaeModeCommand(AlgaeMode.BARGE));

    // controls.algaeModeProcessor().onTrue(superstructure.setAlgaeModeCommand(AlgaeMode.PROCESSOR));

    controls.incrementCoralMode().onTrue(superstructure.incrementCoralModeCommand());

    controls.decrementCoralMode().onTrue(superstructure.decrementCoralModeCommand());

    // Operator

    enableDisablePriorityControl(controls.groundIntakeCoral(), Priority.GROUND_INTAKE_CORAL);
    // enableDisablePriorityControl(controls.groundVomitCoral(), Priority.GROUND_VOMIT_CORAL);
    // enableDisablePriorityControl(controls.sourceIntakeCoral(), Priority.STATION_INTAKE_CORAL);

    // enableDisablePriorityControl(controls.groundIntakeAlgae(), Priority.GROUND_INTAKE_ALGAE);
    // enableDisablePriorityControl(controls.groundVomitAlgae(), Priority.GROUND_VOMIT_ALGAE);
    // enableDisablePriorityControl(controls.stackedIntakeAlgae(), Priority.STACKED_INTAKE_ALGAE);
    // enableDisablePriorityControl(controls.stackedVomitAlgae(), Priority.STACKED_VOMIT_ALGAE);

    enableDisablePriorityControl(controls.prepareScoreCoral(), Priority.PREPARE_SCORE_CORAL);
    // enableDisablePriorityControl(controls.prepareScoreAlgae(), Priority.PREPARE_SCORE_ALGAE);

    // enableDisablePriorityControl(controls.handoffCoral(), Priority.HANDOFF_CORAL);

    // enableDisablePriorityControl(controls.scoreGamePiece(), Priority.SCORE_GAME_PIECE);

    // enableDisablePriorityControl(controls.climb(), Priority.CLIMB);

    // enableDisablePriorityControl(controls.panic(), Priority.PANIC);
    // enableDisablePriorityControl(controls.setManualMode(), Priority.MANUAL);

    // TODO: Use Elevator and Wrist axes as well

    // controls.outtakeShoot().onTrue(Commands.none());

    // controls.algaeClawIntake().onTrue(Commands.none());

    // controls.coralForward().onTrue(Commands.none());

    // controls.coralBackward().onTrue(Commands.none());

    // controls.coralIntakeRun().onTrue(Commands.none());

    // controls.coralIntakeReverse().onTrue(Commands.none());

    // CommandXboxController testController = new CommandXboxController(5);

    // testController.a().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // testController.b().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
    // testController.x().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    // testController.y().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
  }

  /** Handles trigger by enablling priority onTrue and disabling onFalse. */
  public void enableDisablePriorityControl(Trigger trigger, Priority priority) {
    trigger.whileTrue(superstructure.enablePriorityCommand(priority));
    trigger.whileTrue(superstructure.disablePriorityCommand(priority));

    trigger
        .onTrue(superstructure.enablePriorityCommand(priority))
        .onFalse(superstructure.disablePriorityCommand(priority));
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

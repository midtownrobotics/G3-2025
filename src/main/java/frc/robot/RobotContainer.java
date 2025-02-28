// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerIOBag;
import frc.lib.RollerIO.RollerIONeo;
import frc.lib.RollerIO.RollerIOReplay;
import frc.lib.RollerIO.RollerIOSim;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.Controls;
import frc.robot.controls.CoralMode;
import frc.robot.controls.MatchXboxControls;
import frc.robot.sensors.CoralCamera;
import frc.robot.sensors.Photoelectric;
import frc.robot.sensors.vision.VisionIOLimelight;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotIONeo;
import frc.robot.subsystems.coral_intake.pivot.PivotIOReplay;
import frc.robot.subsystems.coral_intake.pivot.PivotIOSim;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.coral_outtake.CoralOuttake.Goal;
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

  // @Getter private final CoralCamera coralCamera;

  @Getter private CANBusStatusSignalRegistration elevatorCANBusHandler = new CANBusStatusSignalRegistration();
  @Getter private CANBusStatusSignalRegistration driveCANBusHandler = new CANBusStatusSignalRegistration();

  private Trigger coralIntakeAtStowGoal;
  private Trigger elevatorAtStowGoal;

  private CoralMode coralMode = CoralMode.L4;

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
    coralOuttake = new CoralOuttake(rollerIO, handoffSensor);
    drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
    
    superstructure = new Superstructure(coralIntake, elevator, coralOuttake);

    new RobotViz(() -> {
      return null;
    }, () -> coralIntake.getPosition(), () -> elevator.getPosition());

    controls = new MatchXboxControls(0, 1);
    configureBindings();

    coralIntakeAtStowGoal = new Trigger(coralIntake.atStowAndStationaryTrigger());
    elevatorAtStowGoal = new Trigger(elevator.atGoalTrigger(Elevator.Goal.STOW));

    coralIntakeAtStowGoal.and(elevatorAtStowGoal).and(controls.handoffCoral()).debounce(0.5).onTrue(Commands.sequence(
      Commands.parallel(
        elevator.setGoalCommand(Elevator.Goal.HANDOFF),
        coralIntake.setGoalCommand(CoralIntake.Goal.HANDOFF),
        coralOuttake.setGoalCommand(CoralOuttake.Goal.HANDOFF)
      ),
      Commands.waitUntil(coralIntake.handoffSensorTrigger),
      Commands.waitSeconds(0.05),
      Commands.waitUntil(coralOuttake.currentSpikeTrigger).withTimeout(2),
      Commands.waitSeconds(0.2),
      coralIntake.setGoalCommand(CoralIntake.Goal.HANDOFF_PUSH_CORAL_UP),
      Commands.waitSeconds(0.2),
      elevator.setGoalCommand(Elevator.Goal.STOW),
      Commands.race(
        Commands.waitUntil(coralIntake.handoffSensorTrigger.negate()),
        Commands.sequence(
          Commands.waitSeconds(2),
          elevator.setGoalCommand(Elevator.Goal.HANDOFF),
          Commands.waitSeconds(0.6),
          elevator.setGoalCommand(Elevator.Goal.STOW)
        ).repeatedly()
      ).withTimeout(10),
      Commands.waitUntil(coralIntake.handoffSensorTrigger.negate()).withTimeout(10),
      Commands.parallel(
        coralOuttake.setGoalCommand(CoralOuttake.Goal.CORAL_BACKWARDS),
        coralIntake.setGoalCommand(CoralIntake.Goal.STOW),
        elevator.setGoalCommand(Elevator.Goal.STOW)
      ),
      Commands.waitSeconds(0.2),
      coralOuttake.setGoalCommand(Goal.IDLE)
    ));

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

    // coralCamera = new CoralCamera(new VisionIOLimelight("limelight", drive::getPose));
  }

  /** Configures bindings to oi */
  private void configureBindings() {
    // Driver

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controls::getDriveForward, controls::getDriveLeft, controls::getDriveRotation));

    controls.resetDriveHeading().onTrue(drive.resetDriveHeadingCommand());

    controls.driveBrake().onTrue(drive.stopWithXCommand());

    // controls.gamePieceLock().onTrue(DriveCommands.joystickDriveAtAngle(drive, controls::getDriveForward, controls::getDriveLeft, () -> {
    //   Logger.recordOutput("AutoIntakeDebug/coralOffset", coralCamera.getCoralOffset().tx());
    //   Logger.recordOutput("AutoIntakeDebug/driveRot", drive.getRotation());
    //   Logger.recordOutput("AutoIntakeDebug/desiredRot", coralCamera.getCoralOffset().tx().plus(drive.getRotation()));
    //   return coralCamera.getCoralOffset().tx().plus(drive.getRotation());
    // }));

    // controls.leftPositionLock().whileTrue(AutoBuilder.7pathfindToPose(new Pose2d(5.27, 3.00, Rotation2d.fromDegrees(120)), new PathConstraints(FeetPerSecond.of(8), FeetPerSecondPerSecond.of(5), DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(480))));

    // controls.groundIntakeCoral().whileTrue(coralIntake.setGoalEndCommand(CoralIntake.Goal.GROUND_INTAKE, CoralIntake.Goal.STOW));

    controls.prepareScoreCoral().whileTrue(elevator.setGoalEndCommand(() -> Elevator.Goal.fromCoralMode(coralMode), Elevator.Goal.STOW));

    controls.scoreGamePiece().whileTrue(coralOuttake.setGoalEndCommand(CoralOuttake.Goal.SHOOT, CoralOuttake.Goal.IDLE));

    controls.prepareScoreCoralL1().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L1));
    controls.prepareScoreCoralL2().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L2));
    controls.prepareScoreCoralL3().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L3));
    controls.prepareScoreCoralL4().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L4));

    controls.scoreGamePiece().whileTrue(coralOuttake.setGoalEndCommand(CoralOuttake.Goal.SHOOT, CoralOuttake.Goal.IDLE));

    // controls.reefAlgaePositionLock().onTrue(Commands.none());

    // controls.algaeModeBarge().onTrue(superstructure.setAlgaeModeCommand(AlgaeMode.BARGE));

    // controls.algaeModeProcessor().onTrue(superstructure.setAlgaeModeCommand(AlgaeMode.PROCESSOR));

    // controls.incrementCoralMode().onTrue(superstructure.incrementCoralModeCommand());

    // controls.decrementCoralMode().onTrue(superstructure.decrementCoralModeCommand());

    // Operator

    // TODO: Use Elevator and Wrist axes as well

    // controls.outtakeShoot().onTrue(Commands.none());

    // controls.algaeClawIntake().onTrue(Commands.none());

    // controls.coralForward().onTrue(Commands.none());

    // controls.coralBackward().onTrue(Commands.none());

    controls.groundIntakeCoral().whileTrue(      
      coralIntake.setGoalCommand(CoralIntake.Goal.GROUND_INTAKE)
    ).onFalse(
      Commands.either(
        Commands.sequence(
          coralIntake.setGoalCommand(CoralIntake.Goal.PRE_HANDOFF_ADJUST_CORAL),
          Commands.waitUntil(coralIntake.atGoalTrigger.and(coralIntake.handoffSensorTrigger.negate())).withTimeout(0.5),
          coralIntake.setGoalCommand(CoralIntake.Goal.STOW)
        ),
        coralIntake.setGoalCommand(CoralIntake.Goal.STOW),
        coralIntake.handoffSensorTrigger
      )
    );

    controls.groundVomitCoral().whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.GROUND_VOMIT, CoralIntake.Goal.STOW) 
    );

    controls.sourceIntakeCoral().whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_INTAKE, null)
    );

    controls.climb().whileTrue(
      elevator.setGoalEndCommand(Elevator.Goal.CLIMB, Elevator.Goal.CLIMB_BOTTOM)
    );

    // controls.coralIntakeReverse().onTrue(Commands.none());

    // CommandXboxController testController = new CommandXboxController(5);

    // testController.a().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // testController.b().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
    // testController.x().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    // testController.y().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
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

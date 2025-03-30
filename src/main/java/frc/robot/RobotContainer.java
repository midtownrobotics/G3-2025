// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.sensors.VisionConstants.kElevatorTagCameraName;
import static frc.robot.sensors.VisionConstants.kModuleTagCameraName;
import static frc.robot.sensors.VisionConstants.kPoleTagCameraName;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AllianceFlipUtil;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerIOKraken;
import frc.lib.RollerIO.RollerIONeo;
import frc.lib.RollerIO.RollerIOReplay;
import frc.lib.RollerIO.RollerIOSim;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.CoralMode;
import frc.robot.controls.MatchXboxControls;
import frc.robot.sensors.Photoelectric;
import frc.robot.sensors.Vision;
import frc.robot.sensors.VisionConstants;
import frc.robot.sensors.vision.VisionIO;
import frc.robot.sensors.vision.VisionIOLimelight;
import frc.robot.sensors.vision.VisionIOSim;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.pivot.PivotIO;
import frc.robot.subsystems.coral_intake.pivot.PivotIONeo;
import frc.robot.subsystems.coral_intake.pivot.PivotIOReplay;
import frc.robot.subsystems.coral_intake.pivot.PivotIOSim;
import frc.robot.subsystems.coral_outtake_pivot.CoralOuttakePivot;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotIO;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotIOKraken;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotIOReplay;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotIOSim;
import frc.robot.subsystems.coral_outtake_roller.CoralOuttakeRoller;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.lock.LockIO;
import frc.robot.subsystems.elevator.lock.LockIORevServo;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchIOKraken;
import frc.robot.subsystems.elevator.winch.WinchIOReplay;
import frc.robot.subsystems.elevator.winch.WinchIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.ReefFace;
import frc.robot.utils.RobotViz;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotContainer {

  private final MatchXboxControls controls;

  private Superstructure superstructure;

  SendableChooser<Command> m_autoChooser;

  @Getter
  private final CoralIntake coralIntake;
  @Getter
  private final CoralOuttakePivot coralOuttakePivot;
  @Getter
  private final CoralOuttakeRoller coralOuttakeRoller;
  @Getter
  private final Elevator elevator;
  @Getter
  private final Drive drive;
  @Getter
  private final Vision aprilTagVision;
  @Getter
  private final LED led;

  // @Getter private final CoralCamera coralCamera;

  @Getter
  private CANBusStatusSignalRegistration elevatorCANBusHandler = new CANBusStatusSignalRegistration("Elevator");
  @Getter
  private CANBusStatusSignalRegistration driveCANBusHandler = new CANBusStatusSignalRegistration("Drivetrain");

  private Trigger coralIntakeAtStowGoal;
  private Trigger elevatorAtStowGoal;

  private Command auto = null;

  @AutoLogOutput
  public CoralMode coralMode = CoralMode.L4;

  /** RobotContainer initialization */
  public RobotContainer() {
    // SignalLogger.start();

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
    OuttakePivotIO outtakePivotIO;

    // Drive
    GyroIO gyroIO;
    ModuleIO flModuleIO;
    ModuleIO frModuleIO;
    ModuleIO blModuleIO;
    ModuleIO brModuleIO;

    VisionIO[] aprilTagVisionIOs;

    led = new LED();
    led.setDefaultCommand(led.applyDefaultPatternCommand());

    switch (Constants.MODE) {
      case REPLAY:
        // Elevator
        winchIO = new WinchIOReplay();

        // Coral Intake
        beltIO = new RollerIOReplay();
        pivotIO = new PivotIOReplay();
        coralIntakeRollerIO = new RollerIOReplay();

        // Coral Outtake
        rollerIO = new RollerIOReplay();
        outtakePivotIO = new OuttakePivotIOReplay();

        // Drive
        gyroIO = new GyroIOPigeon2(driveCANBusHandler);
        flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft, driveCANBusHandler);
        frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight, driveCANBusHandler);
        blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft, driveCANBusHandler);
        brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight, driveCANBusHandler);

        drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);

        aprilTagVision = new Vision(drive::addVisionMeasurement, new VisionIO[0]);
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
        outtakePivotIO = new OuttakePivotIOSim();

        // Drive
        gyroIO = new GyroIOPigeon2(driveCANBusHandler);
        flModuleIO = new ModuleIOSim(TunerConstants.FrontLeft);
        frModuleIO = new ModuleIOSim(TunerConstants.FrontRight);
        blModuleIO = new ModuleIOSim(TunerConstants.BackLeft);
        brModuleIO = new ModuleIOSim(TunerConstants.BackRight);

        drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);

        // Vision
        aprilTagVisionIOs = new VisionIO[] {
            new VisionIOSim("limelight", VisionConstants.kModuleTagRobotToCamera, drive::getPose)
        };

        aprilTagVision = new Vision(drive::addVisionMeasurement, aprilTagVisionIOs);
        break;
      default:
        // Elevator
        winchIO = new WinchIOKraken(Ports.Elevator.LeftWinchMotor,
            Ports.Elevator.RightWinchMotor,
            Ports.Elevator.WinchEncoder,
            driveCANBusHandler);

        // Coral Intake
        beltIO = new RollerIONeo(Ports.CoralIntake.belt, IdleMode.kBrake);
        pivotIO = new PivotIONeo(Ports.CoralIntake.pivotMotor, Ports.CoralIntake.pivotEncoder);
        coralIntakeRollerIO = new RollerIONeo(Ports.CoralIntake.coralIntakeRoller, IdleMode.kBrake);

        // Coral Outtake
        rollerIO = new RollerIOKraken(Ports.CoralOuttake.roller, elevatorCANBusHandler, true);
        outtakePivotIO = new OuttakePivotIOKraken(Ports.CoralOuttake.pivotMotor, Ports.CoralOuttake.pivotEncoder,
            elevatorCANBusHandler);

        // Drive
        gyroIO = new GyroIOPigeon2(driveCANBusHandler);
        flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft, driveCANBusHandler);
        frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight, driveCANBusHandler);
        blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft, driveCANBusHandler);
        brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight, driveCANBusHandler);

        drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);

        aprilTagVisionIOs = new VisionIO[] {
            // new VisionIOPhotonVision("limelight",
            // VisionConstants.kModuleTagRobotToCamera),
            new VisionIOLimelight(kPoleTagCameraName, drive::getRotation),
            new VisionIOLimelight(kModuleTagCameraName, drive::getRotation),
            new VisionIOLimelight(kElevatorTagCameraName, drive::getRotation)
        };

        aprilTagVision = new Vision(drive::addVisionMeasurement, aprilTagVisionIOs);
        break;
    }

    LockIO lockIO = new LockIORevServo(Ports.Elevator.LockServo);
    elevator = new Elevator(winchIO, lockIO);
    coralIntake = new CoralIntake(beltIO, pivotIO, coralIntakeRollerIO, centerSensor, handoffSensor);
    coralOuttakePivot = new CoralOuttakePivot(outtakePivotIO);
    coralOuttakeRoller = new CoralOuttakeRoller(rollerIO);

    superstructure = new Superstructure(coralIntake, elevator, coralOuttakePivot);

    new RobotViz(drive::getPose, coralIntake::getPosition, elevator::getPosition);

    controls = new MatchXboxControls(0, 1);
    configureBindings();

    coralIntakeAtStowGoal = coralIntake.atGoalTrigger(CoralIntake.Goal.STOW, Degrees.of(1.5));
    elevatorAtStowGoal = new Trigger(elevator.atGoalTrigger(Elevator.Goal.STOW));

    // Automatic handoff sequence trigger in teleop
    // RobotModeTriggers.teleop()
    // .and(coralIntakeAtStowGoal)
    // .and(elevatorAtStowGoal)
    // .and(coralIntake.centerSensorTrigger)
    // .and(() -> coralMode != CoralMode.L1)
    // .debounce(0.25)
    // .onTrue(handoffCommand());

    RobotModeTriggers.teleop()
        .and(() -> coralIntake.getCurrentGoal() == CoralIntake.Goal.STOW)
        .and(() -> elevator.getCurrentGoal() == Elevator.Goal.STOW)
        .and(coralIntake.centerSensorTrigger)
        .and(() -> coralMode != CoralMode.L1)
        .debounce(0.25)
        .onTrue(handoffCommand());

    NamedCommands.registerCommand("ScoreCoralLevel4", Commands.sequence(
        elevator.setGoalAndWait(Elevator.Goal.L4).withTimeout(3.0),
        coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.L4).withTimeout(1.0),
        coralOuttakeRoller
            .setGoalEndCommand(CoralOuttakeRoller.Goal.SHOOT, CoralOuttakeRoller.Goal.STOW)
            .withTimeout(0.5),
        elevator.setGoalCommand(Elevator.Goal.STOW)));

    NamedCommands.registerCommand("DisableCameras", aprilTagVision.enableDisableCamera(false, 0));

    NamedCommands.registerCommand("Handoff", handoffCommand());
    // NamedCommands.registerCommand("PrepareLevel4", Commands.none());
    NamedCommands.registerCommand("PrepareLevel4", elevator.setGoalCommand(Elevator.Goal.L4));
    NamedCommands.registerCommand("PrepareLoadingStationIntake",
        coralIntake.setGoalCommand(CoralIntake.Goal.STATION_INTAKE));

    NamedCommands.registerCommand("IntakeFromLoadingStation",
        coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_INTAKE, CoralIntake.Goal.STOW)
            .until(coralIntake.pieceDetectedTrigger.debounce(0.5))
            .withTimeout(2.5));

    NamedCommands.registerCommand("AlignToBranchE", DriveCommands.alignToBranchReef(drive, led, 4, aprilTagVision));
    NamedCommands.registerCommand("AlignToBranchD", DriveCommands.alignToBranchReef(drive, led, 3, aprilTagVision));
    NamedCommands.registerCommand("AlignToBranchK", DriveCommands.alignToBranchReef(drive, led, 10, aprilTagVision));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    auto = AutoBuilder.buildAuto("Bottom Three Coral");

    // coralCamera = new CoralCamera(new VisionIOLimelight("limelight",
    // drive::getPose));
  }

  private double mapInput(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
    return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
  }

  /** Configures bindings to oi */
  private void configureBindings() {
    // Driver

    controls.increaseElevatorOffset().onTrue(Commands.runOnce(() -> {
      elevator.driverOffset = elevator.driverOffset.plus(Inches.of(1));
    }));

    controls.decreaseElevatorOffset().onTrue(Commands.runOnce(() -> {
      elevator.driverOffset = elevator.driverOffset.minus(Inches.of(1));
    }));

    DoubleSupplier speedMultiplier = () -> {
      if (elevator.getPosition().gt(Inches.of(10))) {
        double elevatorHeight = elevator.getPosition().in(Inches);
        return 1.0 - mapInput(elevatorHeight, 10, 60, 0, 0.8);
      }

      return 1.0;
    };

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controls::getDriveForward, controls::getDriveLeft,
        controls::getDriveRotation, speedMultiplier));
    // elevator.setDefaultCommand(new InstantCommand(() -> {
    // if (elevator.getCurrentGoal() != Elevator.Goal.STOW &&
    // elevator.getCurrentGoal() != Elevator.Goal.CLIMB_BOTTOM) {
    // elevator.setGoal(Elevator.Goal.STOW);
    // }
    // }));
    // coralOuttake.setDefaultCommand(new InstantCommand(() -> {
    // coralOuttake.setGoal(CoralOuttake.Goal.STOW);
    // }));
    // coralIntake.setDefaultCommand(new InstantCommand(() -> {
    // coralIntake.setGoal(CoralIntake.Goal.STOW);
    // }));

    controls.resetDriveHeading().onTrue(drive.resetDriveHeadingCommand());

    controls.driveBrake().onTrue(drive.stopWithXCommand());

    controls.panic().whileTrue(new InstantCommand(() -> {
      teleopInit();
    }, coralOuttakePivot, coralOuttakeRoller, coralIntake, elevator));

    RobotModeTriggers.teleop().and(coralIntake.centerSensorTrigger.debounce(0.15)).onTrue(
        led.blinkCommand(Color.kGreen).withTimeout(1.0)
            .alongWith(controls.setRumbleCommand(1.0, Seconds.of(0.4))));

    // controls.gamePieceLock()
    // .whileTrue(DriveCommands.joystickDriveAtAngle(drive,
    // controls::getDriveForward, controls::getDriveLeft, () -> {
    // NetworkTable table =
    // NetworkTableInstance.getDefault().getTable("limelight-ankit");
    // double coralOffset = table.getEntry("tx").getDouble(0.0);

    // if (Math.abs(coralOffset) != 0.0) {
    // coralOffset += 10;
    // }

    // Logger.recordOutput("AutoIntakeDebug/coralOffset", coralOffset);
    // Logger.recordOutput("AutoIntakeDebug/driveRot", drive.getRotation());
    // Logger.recordOutput("AutoIntakeDebug/desiredRot",
    // drive.getRotation().getDegrees() + coralOffset);
    // // return coralCamera.getCoralOffset().tx().plus(drive.getRotation());

    // return drive.getRotation().minus(Rotation2d.fromDegrees(coralOffset));
    // }));

    // controls.leftPositionLock().whileTrue(AutoBuilder.7pathfindToPose(new
    // Pose2d(5.27, 3.00, Rotation2d.fromDegrees(120)), new
    // PathConstraints(FeetPerSecond.of(8), FeetPerSecondPerSecond.of(5),
    // DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(480))));

    // controls.groundIntakeCoral().whileTrue(coralIntake.setGoalEndCommand(CoralIntake.Goal.GROUND_INTAKE,
    // CoralIntake.Goal.STOW));

    controls.prepareScoreCoral().and(() -> coralMode == CoralMode.L1).whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.L1_Prepare, CoralIntake.Goal.STOW));

    controls.prepareScoreCoral().and(() -> coralMode != CoralMode.L1)
        .whileTrue(prepareScoreCoralCommand());

    controls.scoreGamePiece().and(() -> coralMode == CoralMode.L1).whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.L1, CoralIntake.Goal.STOW));

    controls.scoreGamePiece().and(() -> coralMode != CoralMode.L1).whileTrue(
        coralOuttakeRoller.setGoalEndCommand(CoralOuttakeRoller.Goal.SHOOT,
            CoralOuttakeRoller.Goal.STOW));

    controls.reverseGamePiece()
        .whileTrue(coralOuttakeRoller.setGoalEndCommand(CoralOuttakeRoller.Goal.REVERSE_SHOOT,
            CoralOuttakeRoller.Goal.STOW));

    controls.prepareScoreCoralL1().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L1));
    controls.prepareScoreCoralL2().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L2));
    controls.prepareScoreCoralL3().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L3));
    controls.prepareScoreCoralL4().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L4));

    // controls.handoffCoral().onTrue(Commands.parallel(
    // elevator.setGoalCommand(Elevator.Goal.STOW),
    // coralIntake.setGoalCommand(CoralIntake.Goal.STOW),
    // coralOuttake.setGoalCommand(CoralOuttake.Goal.STOW)));

    controls.handoffCoral().onTrue(handoffCommand());

    // controls.reefAlgaePositionLock().onTrue(Commands.none());

    // controls.algaeModeBarge().onTrue(superstructure.setAlgaeModeCommand(AlgaeMode.BARGE));

    // controls.algaeModeProcessor().onTrue(superstructure.setAlgaeModeCommand(AlgaeMode.PROCESSOR));

    // controls.incrementCoralMode().onTrue(superstructure.incrementCoralModeCommand());

    // controls.decrementCoralMode().onTrue(superstructure.decrementCoralModeCommand());

    // Operator

    // TODO: Use Elevator and Wrist axes as well

    // controls.outtakeShoot().onTrue(Commands.none());

    controls.groundIntakeAlgae().onTrue(coralIntake.setGoalCommand(CoralIntake.Goal.ALGAE_INTAKE))
        .onFalse(coralIntake.setGoalCommand(CoralIntake.Goal.HOLD_ALGAE));

    controls.outtakeShoot().and(controls.prepareScoreCoral().negate())
        .and(() -> coralIntake.getCurrentGoal() == CoralIntake.Goal.ALGAE_SHOOT)
        .onTrue(coralIntake.setGoalCommand(CoralIntake.Goal.ALGAE_SHOOT))
        .onFalse(coralIntake.setGoalCommand(CoralIntake.Goal.STOW));

    controls.groundIntakeCoral().whileTrue(
        coralIntake.setGoalCommand(CoralIntake.Goal.GROUND_INTAKE)).onFalse(
            indexCoralAndStowCommand());

    controls.groundVomitCoral().whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.GROUND_VOMIT, CoralIntake.Goal.STOW));

    // Align to reef using the driver's POV
    // controls.alignToReef().whileTrue(DriveCommands.alignToReefFace(drive, led,
    // () -> ReefFace.fromPOV(controls.getDriverPOV()),
    // controls.alignToReefLeftBranch()));

    controls.alignToBranchReef().whileTrue(DriveCommands.alignToBranchReef(drive, led,
        () -> {
          ReefFace closestFace = null;
          Distance closestDistance = Meters.of(Double.MAX_VALUE);
          Pose2d currentPose = drive.getPose();

          for (ReefFace face : ReefFace.values()) {
            Pose2d rawReefFacePose = FieldConstants.Reef.centerFaces[face.ordinal()];
            Pose2d reefFacePose = AllianceFlipUtil.apply(rawReefFacePose);
            Distance distance = Meters.of(reefFacePose.getTranslation().getDistance(currentPose.getTranslation()));
            if (distance.lt(closestDistance)) {
              closestFace = face;
              closestDistance = distance;
            }
          }

          return closestFace;
        }, controls.alignToReefLeftBranch(), aprilTagVision));

    controls.alignToAlgaeReef()
        .whileTrue(DriveCommands.alignToAlgaeReef(drive, led,
            () -> {
              ReefFace closestFace = null;
              Distance closestDistance = Meters.of(Double.MAX_VALUE);
              Pose2d currentPose = drive.getPose();

              for (ReefFace face : ReefFace.values()) {
                Pose2d rawReefFacePose = FieldConstants.Reef.centerFaces[face.ordinal()];
                Pose2d reefFacePose = AllianceFlipUtil.apply(rawReefFacePose);
                Distance distance = Meters.of(reefFacePose.getTranslation().getDistance(currentPose.getTranslation()));
                if (distance.lt(closestDistance)) {
                  closestFace = face;
                  closestDistance = distance;
                }
              }

              return closestFace;
            }))
    // .onFalse(Commands.either(DriveCommands.robotRelativeDrive(drive, () -> -1, ()
    // -> 0, () -> 0).withTimeout(0.5),
    // Commands.none(), () -> !controls.isDriverControlInDeadzone()))
    ;

    controls.gamePieceLock()
        .whileTrue(DriveCommands.alignToGamePiece(drive, controls::getDriveForward, controls::getDriveLeft));

    controls.sourceIntakeCoral().whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_INTAKE, CoralIntake.Goal.STOW));

    controls.sourceIntakeCoral().whileTrue(
        coralIntake.setGoalCommand(CoralIntake.Goal.STATION_INTAKE))
        .onFalse(indexCoralAndStowCommand());

    controls.sourceVomitCoral().whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_VOMIT, CoralIntake.Goal.STOW));

    controls.climb()
        .onTrue(
            Commands.parallel(
                elevator.setGoalCommand(Elevator.Goal.CLIMB),
                coralIntake.setGoalCommand(CoralIntake.Goal.CLIMB),
                coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.CLIMB)))
        .onFalse(
            Commands.sequence(
                coralIntake.setGoalCommand(CoralIntake.Goal.CLIMB_BOTTOM),
                elevator.setGoalAndWait(Elevator.Goal.CLIMB_BOTTOM).withTimeout(2),
                elevator.setGoalCommand(Elevator.Goal.CLIMB_BOTTOM_LOCK)));

    // controls.dealgify()
    // .onTrue(Commands.sequence(
    // elevator.setGoalCommand(elevator.setDealgifyGoalFromCoralMode(() ->
    // coralMode)),
    // Commands.waitUntil(() -> elevator.atGoal(Inches.of(2))),
    // coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.DEALGIFY),
    // coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.SHOOT)
    // ))
    // .onFalse(Commands.sequence(
    // coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.STOW),
    // elevator.setGoalCommand(Elevator.Goal.STOW),
    // coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.STOW)
    // ));

    controls.dealgify()
        .whileTrue(Commands.parallel(
            elevator.setDealgifyGoalFromCoralMode(() -> coralMode),
            Commands.sequence(
                Commands.waitUntil(() -> elevator.atGoal(Inches.of(2))),
                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.DEALGIFY),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.DEALGIFY))))
        .onFalse(Commands.sequence(
            coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.STOW),
            elevator.setGoalCommand(Elevator.Goal.STOW),
            coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.STOW)));

    // CommandXboxController testController = new CommandXboxController(5);

    // testController.a().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // testController.b().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
    // testController.x().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    // testController.y().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
  }

  /** Called when the robot enters teleop */
  public void teleopInit() {
    elevator.setGoal(Elevator.Goal.STOW);
    coralIntake.setGoal(CoralIntake.Goal.STOW);
    coralOuttakePivot.setGoal(CoralOuttakePivot.Goal.STOW);
    coralOuttakeRoller.setGoal(CoralOuttakeRoller.Goal.STOW);
  }

  /** Returns the autonomous command */
  public Command getAutonomousCommand() {

    /*
     * if (auto == null) {
     * return Commands.print("AHHHH");
     * }
     *
     * return auto;
     */
    var selected = m_autoChooser.getSelected();

    if (selected == null) {
      return auto;
    }

    return selected;
  }

  private Command handoffCommand() {
    return Commands.sequence(
        Commands.parallel(
            elevator.setGoalAndWait(Elevator.Goal.STOW),
            coralIntake.setGoalAndWait(CoralIntake.Goal.HANDOFF, Degrees.of(2.5)),
            coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.HANDOFF, Degrees.of(7)),
            coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.HANDOFF)),
        Commands.waitUntil(coralIntake.handoffSensorTrigger).withTimeout(2),
        Commands.waitUntil(coralIntake.handoffSensorTrigger.negate()).withTimeout(2),
        coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.HANDOFF_REVERSE),
        Commands.waitSeconds(0.1),
        coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.STOW).finallyDo(() -> {
          elevator.setGoal(Elevator.Goal.STOW);
          coralIntake.setGoal(CoralIntake.Goal.STOW);
          coralOuttakePivot.setGoal(CoralOuttakePivot.Goal.STOW);
        }));
  }

  /** Returns a command that sets the elevator and coral outtake pivot goals */
  public Command prepareScoreCoralCommand() {
    return Commands.parallel(
        elevator.setGoalEndCommand(() -> Elevator.Goal.fromCoralMode(coralMode), Elevator.Goal.STOW),
        Commands.sequence(
            Commands.waitUntil(() -> elevator.getPosition().gt(Inches.of(5)) && elevator.atGoal(Inches.of(10))),
            coralOuttakePivot.setGoalEndCommand(() -> CoralOuttakePivot.Goal.fromCoralMode(coralMode),
                CoralOuttakePivot.Goal.STOW)));
  }

  /**  */
  public Command indexCoralAndStowCommand() {
    return Commands.either(
        Commands.sequence(
            coralIntake.setGoalCommand(CoralIntake.Goal.PRE_HANDOFF_ADJUST_CORAL),
            Commands.waitUntil(coralIntake.atGoalTrigger.and(coralIntake.centerSensorTrigger)).withTimeout(3.0),
            coralIntake.setGoalCommand(CoralIntake.Goal.STOW)),
        coralIntake.setGoalCommand(CoralIntake.Goal.STOW),
        coralIntake.handoffSensorTrigger);
  }

  private ReefFace getClosestReefFace() {
    ReefFace closestFace = null;
    Distance closestDistance = Meters.of(Double.MAX_VALUE);
    Pose2d currentPose = drive.getPose();

    for (ReefFace face : ReefFace.values()) {
      Pose2d rawReefFacePose = FieldConstants.Reef.centerFaces[face.ordinal()];
      Pose2d reefFacePose = AllianceFlipUtil.apply(rawReefFacePose);
      Distance distance = Meters.of(reefFacePose.getTranslation().getDistance(currentPose.getTranslation()));
      if (distance.lt(closestDistance)) {
        closestFace = face;
        closestDistance = distance;
      }
    }

    return closestFace;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static frc.robot.sensors.VisionConstants.kTagCameraOnAStickRobotToCamera;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerIOKraken;
import frc.lib.RollerIO.RollerIONeo;
import frc.lib.RollerIO.RollerIOReplay;
import frc.lib.RollerIO.RollerIOSim;
import frc.lib.dashboard.LoggedDigitalInput;
import frc.robot.CoralTracker.ReefScorePosition;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.CoralMode;
import frc.robot.controls.SingleDriverControls;
import frc.robot.sensors.Vision;
import frc.robot.sensors.VisionConstants;
import frc.robot.sensors.vision.VisionIO;
import frc.robot.sensors.vision.VisionIOPhotonVision;
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
import frc.robot.utils.AlgaeAction;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import frc.robot.utils.L1Alignment;
import frc.robot.utils.ReefFace;
import frc.robot.utils.ReefFaceSide;
import frc.robot.utils.ReefScoreHeight;
import frc.robot.utils.RobotViz;
import frc.robot.utils.StationSide;
import java.util.Arrays;
import java.util.Set;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

    private final SingleDriverControls controls;

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

    private final CoralTracker coralTracker;

    // @Getter private final CoralCamera coralCamera;

    @Getter
    private CANBusStatusSignalRegistration elevatorCANBusHandler = new CANBusStatusSignalRegistration("Elevator");
    @Getter
    private CANBusStatusSignalRegistration driveCANBusHandler = new CANBusStatusSignalRegistration("Drivetrain");
    @Getter
    private CANBusStatusSignalRegistration rioCANBusHandler = new CANBusStatusSignalRegistration("rio");

    private Trigger coralIntakeAtStowGoal;
    private Trigger elevatorAtStowGoal;

    private Command auto = null;

    @AutoLogOutput
    public CoralMode coralMode = CoralMode.L4;
    @AutoLogOutput
    private CoralMode preL1CoralMode = coralMode;

    private boolean isHandoffInterruptible = true;
    private Trigger waitForHandoffTrigger = new Trigger(() -> isHandoffInterruptible);

    private boolean canStartCoralAlign = true;

    private ReefScorePosition lastAutoScorePosition;

    /** RobotContainer initialization */
    public RobotContainer() {
        // SignalLogger.start();
        SignalLogger.enableAutoLogging(false);

        // Elevator
        WinchIO winchIO;
        LoggedDigitalInput elevatorZeroSensor = new LoggedDigitalInput(Ports.Elevator.zeroSensor);

        // Coral Intake
        RollerIO beltIO;
        PivotIO pivotIO;
        RollerIO coralIntakeRollerIO;
        LoggedDigitalInput centerSensor = new LoggedDigitalInput(Ports.CoralIntake.centerSensor);
        LoggedDigitalInput handoffSensor = new LoggedDigitalInput(Ports.CoralIntake.handoffSensor);
        LoggedDigitalInput intakeUpperZeroSensor = new LoggedDigitalInput(Ports.CoralIntake.upperZeroSensor);
        LoggedDigitalInput intakeLowerZeroSensor = new LoggedDigitalInput(Ports.CoralIntake.lowerZeroSensor);

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

                aprilTagVision = new Vision(drive::getPose, drive::addVisionMeasurement, drive::setPose, new VisionIO[0]);
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
                        new VisionIOSim("limelight", VisionConstants.kModuleTagRobotToCamera,
                                drive::getPose)
                };

                aprilTagVision = new Vision(drive::getPose, drive::addVisionMeasurement, drive::setPose, aprilTagVisionIOs);
                break;
            default:
                // Elevator
                // winchIO = new WinchIOSim();
                winchIO = new WinchIOKraken(Ports.Elevator.LeftWinchMotor,
                        Ports.Elevator.RightWinchMotor,
                        Ports.Elevator.WinchEncoder,
                        driveCANBusHandler);

                // Coral Intake
                beltIO = new RollerIOKraken(Ports.CoralIntake.belt, rioCANBusHandler, true);
                // beltIO = new RollerIOSim();
                pivotIO = new PivotIONeo(Ports.CoralIntake.pivotMotor, Ports.CoralIntake.pivotEncoder);
                // pivotIO = new PivotIOSim();
                coralIntakeRollerIO = new RollerIONeo(Ports.CoralIntake.coralIntakeRoller,
                        IdleMode.kBrake);
                // coralIntakeRollerIO = new RollerIOSim();

                // Coral Outtake
                rollerIO = new RollerIOKraken(Ports.CoralOuttake.roller, elevatorCANBusHandler, true);
                outtakePivotIO = new OuttakePivotIOKraken(Ports.CoralOuttake.pivotMotor,
                        Ports.CoralOuttake.pivotEncoder,
                        elevatorCANBusHandler);

                // Drive
                gyroIO = new GyroIOPigeon2(driveCANBusHandler);
                flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft, driveCANBusHandler);
                frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight, driveCANBusHandler);
                blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft, driveCANBusHandler);
                brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight, driveCANBusHandler);

                drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);

                aprilTagVisionIOs = new VisionIO[] {
                        new VisionIOPhotonVision("Intake_Module_Cam", kTagCameraOnAStickRobotToCamera),
                        // new VisionIOLimelight(kModuleTagCameraName, drive::getRotation,
                                // kModuleTagRobotToCamera, kModuleTagCameraPositionCorrection),
                        // new VisionIOLimelight(kTagCameraOnAStickCameraName, drive::getRotation,
                                // kTagCameraOnAStickRobotToCamera, new Transform3d())
                };

                aprilTagVision = new Vision(drive::getPose, drive::addVisionMeasurement, drive::setPose, aprilTagVisionIOs);
                break;
        }

        LockIO lockIO = new LockIORevServo(Ports.Elevator.LockServo);
        elevator = new Elevator(winchIO, lockIO, elevatorZeroSensor);
        coralIntake = new CoralIntake(beltIO, pivotIO, coralIntakeRollerIO, centerSensor, handoffSensor,
                intakeUpperZeroSensor, intakeLowerZeroSensor);
        coralOuttakePivot = new CoralOuttakePivot(outtakePivotIO);
        coralOuttakeRoller = new CoralOuttakeRoller(rollerIO);

        superstructure = new Superstructure(coralIntake, elevator, coralOuttakePivot);

        coralTracker = new CoralTracker(() -> ReefFace.getClosestReefFace(drive));
        lastAutoScorePosition = coralTracker.getBestNearScorePosition();

        new RobotViz(drive::getPose, coralIntake::getPosition, elevator::getPosition);

        controls = new SingleDriverControls(0);
        configureBindings();

        coralIntakeAtStowGoal = coralIntake.atGoalTrigger(CoralIntake.Goal.STOW, Degrees.of(1.5));
        elevatorAtStowGoal = new Trigger(elevator.atGoalTrigger(Elevator.Goal.STOW));

        RobotModeTriggers.teleop()
                .and(() -> coralIntake.getCurrentGoal() == CoralIntake.Goal.STOW)
                .and(() -> elevator.getCurrentGoal() == Elevator.Goal.STOW)
                .and(coralIntake.centerSensorTrigger)
                .and(() -> coralMode != CoralMode.L1)
                .debounce(0.25)
                .onTrue(handoffCommand());

        NamedCommands.registerCommand("ScoreCoralLevel4", Commands.sequence(
                prepareScoreCoral(CoralMode.L4),
                coralOuttakeRoller
                        .setGoalEndCommand(CoralOuttakeRoller.Goal.SHOOT_L4,
                                CoralOuttakeRoller.Goal.STOW)
                        .withTimeout(0.35),
                Commands.parallel(
                        coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.STOW),
                        elevator.setGoalCommand(Elevator.Goal.STOW))));

        NamedCommands.registerCommand("PrepareLevel1", coralIntake.setGoalCommand(CoralIntake.Goal.L1_PREPARE));
        NamedCommands.registerCommand("ScoreCoralLevel1", coralIntake.setGoalCommand(CoralIntake.Goal.L1));

        NamedCommands.registerCommand("DisableCameras", aprilTagVision.enableDisableCamera(false, 0));

        NamedCommands.registerCommand("Handoff", handoffCommand());
        NamedCommands.registerCommand("WaitForHandoff", Commands.waitUntil(waitForHandoffTrigger));
        // NamedCommands.registerCommand("PrepareLevel4",
        // prepareScoreCoral(CoralMode.L4));
        NamedCommands.registerCommand("PrepareLevel4", Commands.sequence(
                Commands.waitUntil(waitForHandoffTrigger),
                Commands.defer(() -> prepareScoreCoral(CoralMode.L4), Set.of())));
        NamedCommands.registerCommand("PrepareLoadingStationIntake",
                coralIntake.setGoalCommand(CoralIntake.Goal.STATION_INTAKE)
                        .alongWith(coralOuttakePivot
                                .setGoalCommand(CoralOuttakePivot.Goal.L2)));

        NamedCommands.registerCommand("IntakeFromGround",
                coralIntake.setGoalCommand(CoralIntake.Goal.GROUND_INTAKE));

        NamedCommands.registerCommand("IntakeFromLoadingStation",
                Commands.race(
                        coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_INTAKE,
                                CoralIntake.Goal.STOW)
                                .until(coralIntake.pieceDetectedTrigger.debounce(0.15))
                                .withTimeout(7),
                        DriveCommands.robotRelativeDrive(drive, () -> 0, () -> 0.2, () -> 0)));

        NamedCommands.registerCommand("PrepareIntakeAlgaeLow", Commands.sequence(
                elevator.setGoalAndWait(Elevator.Goal.DEALGIFY_LOW, Inches.of(2.5)),
                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.DEALGIFY, Degrees.of(4)),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.DEALGIFY)));

        NamedCommands.registerCommand("PrepareIntakeAlgaeHigh", Commands.sequence(
                elevator.setGoalAndWait(Elevator.Goal.DEALGIFY_HIGH, Inches.of(2.5)),
                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.DEALGIFY, Degrees.of(4)),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.DEALGIFY)));

        NamedCommands.registerCommand("IntakeAlgaeLow", Commands.sequence(
                elevator.setGoalAndWait(Elevator.Goal.DEALGIFY_LOW),
                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.DEALGIFY),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.DEALGIFY),
                Commands.waitSeconds(0.3),
                Commands.waitUntil(coralOuttakeRoller.currentSpikeTrigger).withTimeout(2.5),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_HOLD)));

        NamedCommands.registerCommand("AlgaeStow", Commands.parallel(
                coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.DEALGIFY_STOW),
                elevator.setGoalCommand(Elevator.Goal.STOW),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_HOLD)));

        NamedCommands.registerCommand("AlgaeHold",
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_HOLD));

        NamedCommands.registerCommand("PrepareBargeShoot", Commands.sequence(
                elevator.setGoalAndWait(Elevator.Goal.BARGE, Inches.of(4)),
                coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.BARGE)));

        NamedCommands.registerCommand("BargeShoot", Commands.sequence(
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_SHOOT)));

        NamedCommands.registerCommand("Stow", Commands.runOnce(() -> {
            teleopInit();
        }).withTimeout(1));

        // barge
        // controls.algae().and(() -> (coralMode == CoralMode.L4))
        // .whileTrue(Commands.sequence(
        // elevator.setGoalAndWait(Elevator.Goal.BARGE),
        // coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.BARGE)))
        // .onFalse(Commands.parallel(
        // elevator.setGoalCommand(Elevator.Goal.STOW),
        // coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.STOW)));

        // controls.algae().and(controls.scoreGamePiece()).whileTrue(
        NamedCommands.registerCommand("AlignToBranchA",
                DriveCommands.alignToBranchReef(drive, led, 0).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchB",
                DriveCommands.alignToBranchReef(drive, led, 1).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchC",
                DriveCommands.alignToBranchReef(drive, led, 2).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchD",
                DriveCommands.alignToBranchReef(drive, led, 3).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchE",
                DriveCommands.alignToBranchReef(drive, led, 4).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchF",
                DriveCommands.alignToBranchReef(drive, led, 5).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchG",
                DriveCommands.alignToBranchReef(drive, led, 6).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchH",
                DriveCommands.alignToBranchReef(drive, led, 7).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchI",
                DriveCommands.alignToBranchReef(drive, led, 8).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchJ",
                DriveCommands.alignToBranchReef(drive, led, 9).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchK",
                DriveCommands.alignToBranchReef(drive, led, 10).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToBranchL",
                DriveCommands.alignToBranchReef(drive, led, 11).withTimeout(1.6));

        NamedCommands.registerCommand("AlignToAlgae4",
                DriveCommands.alignToAlgaeReef(drive, led, () -> ReefFace.GH, () -> false)
                        .withTimeout(1.6));
        NamedCommands.registerCommand("AlignToAlgae5",
                DriveCommands.alignToAlgaeReef(drive, led, () -> ReefFace.IJ, () -> false)
                        .withTimeout(1.6));

        NamedCommands.registerCommand("PrepareLevel1", Commands.runOnce(() -> coralMode = CoralMode.L1));

        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        auto = AutoBuilder.buildAuto("Bottom AA 3 Piece v2");

        // coralCamera = new CoralCamera(new VisionIOLimelight("limelight",
        // drive::getPose));
    }

    private Command prepareScoreCoral(CoralMode mode) {
        return Commands.sequence(
                elevator.setGoalAndWait(Elevator.Goal.fromCoralMode(mode), Inches.of(5)),
                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.fromCoralMode(mode),
                        Degrees.of(4)),
                elevator.setGoalAndWait(Elevator.Goal.fromCoralMode(mode)).withTimeout(0.4),
                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.fromCoralMode(mode))
                        .withTimeout(0.4));
    }

    private double mapInput(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
        return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
    }

    /** Configures bindings to oi */
    private void configureBindings() {

        // DriveToPoint.overrideTriggger = controls.manualOverride();

        DoubleSupplier speedMultiplier = () -> {
            if (elevator.getPosition().gt(Inches.of(10))) {
                double elevatorHeight = elevator.getPosition().in(Inches);
                double maxSpeed = coralOuttakePivot.getCurrentGoal() == CoralOuttakePivot.Goal.BARGE
                        ? 0.7
                        : 0.8;
                return 1.0 - mapInput(elevatorHeight, 10, 60, 0, maxSpeed);
            }

            return 1.0;
        };

        drive.setDefaultCommand(
                DriveCommands.joystickDrive(drive, controls::getDriveForward, controls::getDriveLeft,
                        controls::getDriveRotation, speedMultiplier));

        controls.setAutoCoralMode().onTrue(Commands.runOnce(() -> coralMode = CoralMode.AUTO));
        controls.prepareScoreCoralL2().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L2));
        controls.prepareScoreCoralL3().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L3));
        controls.prepareScoreCoralL4().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L4));

        controls.algaeAndL1CenterAutoAlign()
                .and(() -> AlgaeAction.REEF.shouldDo(drive, () -> coralMode))
                .debounce(0.05)
                .onTrue(Commands.sequence(
                        Commands.parallel(
                                elevator.setGoalCommand(
                                        () -> ReefFace.getClosestReefFace(drive).isAlgaePositionHigh()
                                                ? Elevator.Goal.DEALGIFY_HIGH
                                                : Elevator.Goal.DEALGIFY_LOW),
                                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.DEALGIFY),
                                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.DEALGIFY),
                                DriveCommands.alignToAlgaeReef(drive, led,
                                        () -> ReefFace.getClosestReefFace(drive),
                                        () -> true)
                        ),
                        Commands.waitUntil(() -> elevator.atGoal(Inches.of(2))),
                        DriveCommands.alignToAlgaeReef(drive, led,
                                () -> ReefFace.getClosestReefFace(drive),
                                () -> false).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
                        Commands.waitSeconds(2).until(coralOuttakeRoller.currentSpikeTrigger),
                        coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_HOLD),
                        DriveCommands.alignToAlgaeReef(drive, led,
                                () -> ReefFace.getClosestReefFace(drive),
                                () -> true)
                ).finallyDo(() -> {
                        coralOuttakeRoller.setGoal(CoralOuttakeRoller.Goal.ALGAE_HOLD);
                        coralOuttakePivot.setGoal(CoralOuttakePivot.Goal.DEALGIFY_STOW);
                        elevator.setGoal(() -> ReefFace.getClosestReefFace(drive).isAlgaePositionHigh()
                                ? Elevator.Goal.DEALGIFY_HIGH
                                : Elevator.Goal.DEALGIFY_LOW);
                }))
                .onFalse(Commands.sequence(
                        DriveCommands.alignToAlgaeReef(drive, led,
                        () -> ReefFace.getClosestReefFace(drive),
                        () -> true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
                        AlgaeAction.setHasAlgae(true),
                        Commands.parallel(
                                coralOuttakePivot.setGoalCommand(
                                        CoralOuttakePivot.Goal.DEALGIFY_STOW),
                                // coralOuttakeRoller.setGoalCommand(
                                //         CoralOuttakeRoller.Goal.ALGAE_HOLD),
                                Commands.sequence(
                                        Commands.runOnce(
                                                () -> canStartCoralAlign = false),
                                        Commands.waitTime(Milliseconds.of(500)),
                                        Commands.runOnce(
                                                () -> canStartCoralAlign = true)))));

        controls.bargeInitialAlign()
                .and(() -> AlgaeAction.BARGE.shouldDo(drive, () -> coralMode))
                .debounce(0.05)
                .whileTrue(
                        // DriveCommands.alignToBarge(drive, controls::getDriveLeft,
                        //         Commands.sequence(
                        //                 elevator.setGoalAndWait(Elevator.Goal.BARGE, Inches.of(2)),
                        //                 coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.BARGE)
                        //         )
                        // )
                        Commands.parallel(
                                elevator.setGoalAndWait(Elevator.Goal.BARGE, Inches.of(2)),
                                coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.BARGE)
                        )
                );

        controls.algaeAndL1CenterAutoAlign()
                .and(() -> AlgaeAction.BARGE.shouldDo(drive, () -> coralMode))
                .debounce(0.05)
                .whileTrue(
                        Commands.sequence(
                                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.BARGE),
                                // new DriveToPoint(
                                //         drive,
                                //         () -> new Pose2d(
                                //                 new Translation2d(
                                //                         Meters.of(7.85),
                                //                         drive.getPose().getMeasureY()
                                //                 ),
                                //                 new Rotation2d()
                                //         ),
                                //         Degrees.of(5),
                                //         Inches.of(2)
                                // ),
                                // new DriveToX(drive, () -> Meters.of(8.05), () -> 0.0, () -> Degrees.of(0), Degrees.of(5), Inches.of(2)),
                                // new InstantCommand(() -> drive.stop(), drive),
                                elevator.setGoalAndWait(Elevator.Goal.BARGE, Inches.of(2)),
                                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_SHOOT),
                                Commands.waitSeconds(2)
                        )
                )
                .onFalse(
                        Commands.sequence(
                                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.STOW),
                                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.STOW),
                                // new DriveToPoint(
                                //         drive,
                                //         () -> new Pose2d(
                                //                 new Translation2d(
                                //                         Meters.of(7),
                                //                         drive.getPose().getMeasureY()
                                //                 ),
                                //                 new Rotation2d()
                                //         ),
                                //         Degrees.of(5),
                                //         Inches.of(2)
                                // ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
                                elevator.setGoalCommand(Elevator.Goal.STOW),
                                AlgaeAction.setHasAlgae(false)
                        )
                );

        controls.algaeAndL1CenterAutoAlign()
                .and(() -> AlgaeAction.PROCESSOR.shouldDo(drive, () -> coralMode))
                .debounce(0.05)
                .whileTrue(Commands.sequence(
                        Commands.parallel(
                                DriveCommands.alignToProcessor(drive, true),
                                elevator.setGoalAndWait(Elevator.Goal.PROCESSOR),
                                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.PROCESSOR_SCORE)
                        ),
                        coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_SHOOT),
                        Commands.waitSeconds(0.25),
                        coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.STOW),
                        coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.STOW),
                        DriveCommands.alignToProcessor(drive, false)
                ))
                .onFalse(Commands.parallel(
                        coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.STOW),
                        // DriveCommands.alignToProcessor(drive, true),
                        coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.STOW),
                        AlgaeAction.setHasAlgae(false)
                ));

        controls.coralAutoAlign()
                .and(() -> coralMode != CoralMode.L1)
                .and(() -> !AlgaeAction.BARGE.shouldDo(drive, () -> coralMode))
                .and(() -> canStartCoralAlign)
                .debounce(0.05)
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.alignToBranchReef(drive, led,
                                        () -> ReefFace.getClosestReefFace(drive),
                                        controls.branchSelectedSupplier(),
                                        () -> false),
                                Commands.sequence(
                                        Commands.waitUntil(() -> drive
                                                .isWithinToleranceToPose(
                                                        DriveCommands.getRobotAlignBranchPoseFromReefFace(
                                                                () -> ReefFace.getClosestReefFace(drive),
                                                                controls.branchSelectedSupplier()),
                                                        Feet.of(3),
                                                        Degrees.of(180))),
                                        elevator.setGoalAndWait(
                                                () -> Elevator.Goal
                                                        .fromCoralMode(coralMode)),
                                        coralOuttakePivot
                                                .setGoalAndWait(() -> CoralOuttakePivot.Goal
                                                        .fromCoralMode(coralMode)),
                                        coralOuttakeRoller.setGoalCommand(
                                                () -> CoralOuttakeRoller.Goal
                                                        .fromCoralMode(coralMode))),
                                Commands.runOnce(() -> coralTracker.addCoralScored(
                                        ReefFace.getClosestReefFace(drive), controls.branchSelectedSupplier().get(),
                                        ReefScoreHeight.fromCoralMode(coralMode)))))
                .onFalse(
                        Commands.sequence(
                                coralOuttakePivot.setGoalAndWait(
                                        CoralOuttakePivot.Goal.STOW),
                                elevator.setGoalCommand(Elevator.Goal.STOW),
                                coralOuttakeRoller.setGoalCommand(
                                        CoralOuttakeRoller.Goal.STOW)));

        controls.coralAutoAlign()
                .onTrue(Commands.runOnce(() -> {
                    lastAutoScorePosition = coralTracker.getBestNearScorePosition();

                    Logger.recordOutput("CoralTracker/Position/Face", lastAutoScorePosition.face);
                    Logger.recordOutput("CoralTracker/Position/Side", lastAutoScorePosition.side);
                    Logger.recordOutput("CoralTracker/Position/Height", lastAutoScorePosition.height);
                }));

        controls.coralAutoAlign()
                .and(() -> coralMode == CoralMode.AUTO)
                .and(() -> canStartCoralAlign)
                .debounce(0.05)
                .whileTrue(
                        Commands.either(Commands.none(), Commands.parallel(
                                DriveCommands.alignToBranchReef(drive, led,
                                        () -> lastAutoScorePosition.face,
                                        () -> lastAutoScorePosition.side,
                                        () -> false),
                                Commands.sequence(
                                        Commands.waitUntil(() -> drive
                                                .isWithinToleranceToPose(
                                                        DriveCommands.getRobotAlignBranchPoseFromReefFace(
                                                                () -> lastAutoScorePosition.face,
                                                                () -> lastAutoScorePosition.side),
                                                        Feet.of(3),
                                                        Degrees.of(180))),
                                        elevator.setGoalAndWait(
                                                () -> Elevator.Goal
                                                        .fromCoralMode(lastAutoScorePosition.height
                                                                .toCoralMode())),
                                        coralOuttakePivot
                                                .setGoalAndWait(() -> CoralOuttakePivot.Goal
                                                        .fromCoralMode(lastAutoScorePosition.height
                                                                .toCoralMode())),
                                        coralOuttakeRoller
                                                .setGoalCommand(() -> CoralOuttakeRoller.Goal
                                                        .fromCoralMode(lastAutoScorePosition.height
                                                                .toCoralMode())),
                                        Commands.runOnce(
                                                () -> coralTracker
                                                        .addCoralScored(lastAutoScorePosition)))),
                                () -> lastAutoScorePosition.height == ReefScoreHeight.L1))
                .onFalse(
                        Commands.sequence(
                                coralOuttakePivot.setGoalAndWait(
                                        CoralOuttakePivot.Goal.STOW),
                                elevator.setGoalCommand(Elevator.Goal.STOW),
                                coralOuttakeRoller.setGoalCommand(
                                        CoralOuttakeRoller.Goal.STOW)));

        controls.coralAutoAlign()
                .or(controls.algaeAndL1CenterAutoAlign())
                .and(() -> coralMode == CoralMode.L1)
                .and(() -> AlgaeAction.NONE.shouldDo(drive, () -> coralMode))
                .and(() -> canStartCoralAlign)
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.alignToL1Reef(drive, led,
                                        () -> ReefFace.getClosestReefFace(drive), this::getIndicatedL1Alignment),
                                Commands.sequence(
                                        Commands.waitUntil(
                                                () -> drive.isWithinToleranceToPose(
                                                        DriveCommands.getRobotAlignL1FacePoseFromReefFace(
                                                                () -> ReefFace.getClosestReefFace(drive), this::getIndicatedL1Alignment),
                                                        Feet.of(0.2),
                                                        Degrees.of(15))),
                                        coralIntake.setGoalAndWait(
                                                CoralIntake.Goal.L1))))
                .onFalse(
                        coralIntake.setGoalCommand(CoralIntake.Goal.STOW));

        controls.manualShoot().and(controls.coralAutoAlign().or(controls.algaeAndL1CenterAutoAlign()))
                .whileTrue(
                        coralOuttakeRoller.setGoalEndCommand(
                                () -> CoralOuttakeRoller.Goal.fromCoralMode(coralMode),
                                CoralOuttakeRoller.Goal.STOW));

        controls.manualOverride().whileTrue(
                Commands.sequence(
                        elevator.setGoalAndWait(
                                                () -> Elevator.Goal
                                                        .fromCoralMode(coralMode)),
                                        coralOuttakePivot
                                                .setGoalAndWait(() -> CoralOuttakePivot.Goal
                                                        .fromCoralMode(coralMode)),
                                        coralOuttakeRoller.setGoalCommand(
                                                () -> CoralOuttakeRoller.Goal
                                                        .fromCoralMode(coralMode))
                )
        ).onFalse(Commands.sequence(
                elevator.setGoalCommand(Elevator.Goal.STOW),
                coralOuttakePivot
                        .setGoalCommand(CoralOuttakePivot.Goal.STOW),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal
                                .STOW)
        ));

        controls.intake().and(controls.coralAutoAlign().or(controls.algaeAndL1CenterAutoAlign()).negate())
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> {
                                    if (coralMode == CoralMode.L1) {
                                        coralMode = preL1CoralMode;
                                    }
                                    preL1CoralMode = coralMode;
                                }),
                                Commands.either(coralIntake.setGoalCommand(
                                        CoralIntake.Goal.GROUND_INTAKE),
                                        Commands.parallel(coralIntake
                                                .setGoalCommand(CoralIntake.Goal.STATION_INTAKE),
                                                DriveCommands.alignToStation(
                                                        drive,
                                                        led,
                                                        this::getClosestStation)),
                                        controls.coralIntakeModeSupplier())))
                .onFalse(indexCoralAndStowCommand());

        controls.intakeL1().and(controls.coralAutoAlign().or(controls.algaeAndL1CenterAutoAlign()).negate())
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> {
                                    preL1CoralMode = (coralMode != CoralMode.L1)
                                            ? coralMode
                                            : preL1CoralMode;
                                    coralMode = CoralMode.L1;
                                }),
                                Commands.either(coralIntake.setGoalCommand(
                                        CoralIntake.Goal.GROUND_INTAKE),
                                        Commands.parallel(coralIntake
                                                .setGoalCommand(CoralIntake.Goal.STATION_INTAKE),
                                                DriveCommands.alignToStation(
                                                        drive,
                                                        led,
                                                        this::getClosestStation)),
                                        controls.coralIntakeModeSupplier())))
                .onFalse(indexCoralAndStowCommand());

        controls.climb()
                .onTrue(
                        Commands.parallel(
                                elevator.setGoalCommand(Elevator.Goal.CLIMB),
                                coralIntake.setGoalCommand(CoralIntake.Goal.CLIMB),
                                coralOuttakePivot.setGoalCommand(
                                        CoralOuttakePivot.Goal.CLIMB)))
                .onFalse(
                        Commands.sequence(
                                coralIntake.setGoalCommand(
                                        CoralIntake.Goal.CLIMB_BOTTOM),
                                elevator.setGoalAndWait(Elevator.Goal.CLIMB_BOTTOM)
                                        .withTimeout(2.2),
                                elevator.setGoalCommand(
                                        Elevator.Goal.CLIMB_BOTTOM_LOCK)));

        controls.increaseElevatorOffset()
                .onTrue(Commands.runOnce(() -> {
                    elevator.driverOffset = elevator.driverOffset.plus(Inches.of(0.5));
                }));

        controls.decreaseElevatorOffset()
                .onTrue(Commands.runOnce(() -> {
                    elevator.driverOffset = elevator.driverOffset.minus(Inches.of(0.5));
                }));

        controls.handoffCoral().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> coralMode = preL1CoralMode),
                        handoffCommand()));

        controls.eject().and(() -> isHandoffInterruptible)
                .and(() -> !AlgaeAction.hasAlgae)
                .onTrue(
                        Commands.parallel(
                                coralIntake.setGoalCommand(
                                        CoralIntake.Goal.STATION_VOMIT),
                                coralOuttakeRoller.setGoalCommand(
                                        CoralOuttakeRoller.Goal.SHOOT_L4)))
                .onFalse(
                        Commands.parallel(
                                coralIntake.setGoalCommand(
                                        CoralIntake.Goal.STOW),
                                coralOuttakeRoller.setGoalCommand(
                                        CoralOuttakeRoller.Goal.STOW))
                );

        controls.eject()
                .and(() -> AlgaeAction.hasAlgae)
                .onTrue(
                        Commands.parallel(
                                coralIntake.setGoalEndCommand(
                                        CoralIntake.Goal.STATION_VOMIT,
                                        CoralIntake.Goal.STOW),
                                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_SHOOT)
                        ))
                .onFalse(
                        AlgaeAction.setHasAlgae(false)
                );

        controls.reset().whileTrue(new InstantCommand(() -> {
            teleopInit();
        }, coralOuttakePivot, coralOuttakeRoller, coralIntake, elevator));

        RobotModeTriggers.teleop().and(coralIntake.centerSensorTrigger.debounce(0.15)).onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> controls.setRumble(1)),
                        new WaitCommand(0.312),
                        Commands.runOnce(() -> controls.setRumble(0))
                )
        );
    }

    /** Called when the robot enters teleop */
    public void teleopInit() {
        elevator.setGoal(Elevator.Goal.STOW);
        coralIntake.setGoal(CoralIntake.Goal.STOW);
        coralOuttakePivot.setGoal(AlgaeAction.hasAlgae ? CoralOuttakePivot.Goal.DEALGIFY_STOW : CoralOuttakePivot.Goal.STOW);
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
                        Commands.runOnce(() -> {
                            isHandoffInterruptible = false;
                        }),
                        elevator.setGoalAndWait(Elevator.Goal.STOW),
                        coralIntake.setGoalAndWait(CoralIntake.Goal.HANDOFF, Degrees.of(3.5)),
                        coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.HANDOFF,
                                Degrees.of(20)))
                        .withTimeout(1.0),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.HANDOFF),
                Commands.waitUntil(coralIntake.handoffSensorTrigger).withTimeout(2),
                Commands.waitUntil(coralIntake.handoffSensorTrigger.negate()).withTimeout(2),
                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.HANDOFF_REVERSE),
                Commands.waitSeconds(0.1)).finallyDo(() -> {
                    elevator.setGoal(Elevator.Goal.STOW);
                    coralIntake.setGoal(CoralIntake.Goal.STOW);
                    coralOuttakePivot.setGoal(CoralOuttakePivot.Goal.STOW);
                    coralOuttakeRoller.setGoal(CoralOuttakeRoller.Goal.STOW);
                    isHandoffInterruptible = true;
                });
    }

    /** Returns a command that sets the elevator and coral outtake pivot goals */
    public Command prepareScoreCoralCommand() {
        return Commands.parallel(
                elevator.setGoalEndCommand(() -> Elevator.Goal.fromCoralMode(coralMode),
                        Elevator.Goal.STOW),
                Commands.sequence(
                        Commands.waitUntil(
                                () -> (elevator.getPosition().gt(Inches.of(5))
                                        && elevator.atGoal(Inches.of(10)))),
                        coralOuttakePivot.setGoalEndCommand(
                                () -> CoralOuttakePivot.Goal.fromCoralMode(coralMode),
                                CoralOuttakePivot.Goal.STOW)))
                .finallyDo(() -> coralOuttakePivot.setGoal(CoralOuttakePivot.Goal.STOW));
    }

    /**  */
    public Command indexCoralAndStowCommand() {
        return Commands.either(
                Commands.sequence(
                        coralIntake.setGoalCommand(CoralIntake.Goal.PRE_HANDOFF_ADJUST_CORAL),
                        Commands.waitUntil(coralIntake.atGoalTrigger
                                .and(coralIntake.centerSensorTrigger))
                                .withTimeout(3.0),
                        coralIntake.setGoalCommand(CoralIntake.Goal.STOW)),
                coralIntake.setGoalCommand(
                        coralMode == CoralMode.L1 ? CoralIntake.Goal.L1_PREPARE
                                : CoralIntake.Goal.STOW),
                coralIntake.handoffSensorTrigger);
    }

    private StationSide getClosestStation() {
        StationSide closestStation = null;
        Distance closestDistance = Meters.of(Double.MAX_VALUE);

        for (StationSide side : StationSide.values()) {
            Pose2d stationPose;

            switch (side) {
                case RIGHT:
                    stationPose = new Pose2d(Meters.of(1.548), Meters.of(0.767),
                            new Rotation2d(edu.wpi.first.math.util.Units
                                    .degreesToRadians(140)));
                    break;
                case LEFT:
                    stationPose = new Pose2d(Meters.of(1.336), Meters.of(7.116),
                            new Rotation2d(edu.wpi.first.math.util.Units
                                    .degreesToRadians(220)));
                    break;
                default:
                    stationPose = null;
            }

            Distance distance = Meters
                    .of(stationPose.getTranslation().getDistance(drive.getPose().getTranslation()));
            if (distance.lt(closestDistance)) {
                closestDistance = distance;
                closestStation = side;
            }
        }

        Logger.recordOutput("/closestStation", closestStation);

        return closestStation;
    }

    @AutoLogOutput
    private L1Alignment getIndicatedL1Alignment() {
        if (controls.algaeAndL1CenterAutoAlign().getAsBoolean()) { return L1Alignment.CENTER; }
        if (controls.branchSelectedSupplier().get() == ReefFaceSide.LEFT) { return L1Alignment.LEFT; }
        if (controls.branchSelectedSupplier().get() == ReefFaceSide.RIGHT ) { return L1Alignment.RIGHT; }
        return L1Alignment.CENTER;
    }

    /** used for logging */
    public void periodic() {
        Logger.recordOutput("FinishedZeroing",
                coralIntake.getZeroSensorDebounced(true) && elevator.getZeroSensorDebounced());

        Logger.recordOutput("CoralTracker/ScoredCorals", Arrays.deepToString(coralTracker.scoredCorals));
        Logger.recordOutput("GrayTesting/AlgaeAction", AlgaeAction.getBestContexually(drive, () -> coralMode));
    }
}

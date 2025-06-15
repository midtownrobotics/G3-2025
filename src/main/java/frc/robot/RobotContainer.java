// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static frc.robot.sensors.VisionConstants.kElevatorTagCameraName;
import static frc.robot.sensors.VisionConstants.kElevatorTagRobotToCamera;
import static frc.robot.sensors.VisionConstants.kModuleTagCameraName;
import static frc.robot.sensors.VisionConstants.kModuleTagRobotToCamera;
import static frc.robot.sensors.VisionConstants.kPoleTagCameraName;
import static frc.robot.sensors.VisionConstants.kPoleTagRobotToCamera;

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
import frc.lib.dashboard.LoggedDigitalInput;
import frc.robot.commands.DriveCommands;
import frc.robot.controls.CoralMode;
import frc.robot.controls.SingleDriverControls;
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
import frc.robot.utils.StationSide;
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
                        new VisionIOSim("limelight", VisionConstants.kModuleTagRobotToCamera,
                                drive::getPose)
                };

                aprilTagVision = new Vision(drive::addVisionMeasurement, aprilTagVisionIOs);
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
                        // new VisionIOPhotonVision("limelight",
                        // VisionConstants.kModuleTagRobotToCamera),
                        new VisionIOLimelight(kPoleTagCameraName, drive::getRotation,
                                kPoleTagRobotToCamera),
                        new VisionIOLimelight(kModuleTagCameraName, drive::getRotation,
                                kModuleTagRobotToCamera),
                        new VisionIOLimelight(kElevatorTagCameraName, drive::getRotation,
                                kElevatorTagRobotToCamera)
                };

                aprilTagVision = new Vision(drive::addVisionMeasurement, aprilTagVisionIOs);
                break;
        }

        LockIO lockIO = new LockIORevServo(Ports.Elevator.LockServo);
        elevator = new Elevator(winchIO, lockIO, elevatorZeroSensor);
        coralIntake = new CoralIntake(beltIO, pivotIO, coralIntakeRollerIO, centerSensor, handoffSensor,
                intakeUpperZeroSensor, intakeLowerZeroSensor);
        coralOuttakePivot = new CoralOuttakePivot(outtakePivotIO);
        coralOuttakeRoller = new CoralOuttakeRoller(rollerIO);

        superstructure = new Superstructure(coralIntake, elevator, coralOuttakePivot);

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
                DriveCommands.alignToAlgaeReef(drive, led, () -> ReefFace.GH, () -> false).withTimeout(1.6));
        NamedCommands.registerCommand("AlignToAlgae5",
                DriveCommands.alignToAlgaeReef(drive, led, () -> ReefFace.IJ, () -> false).withTimeout(1.6));

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

        controls.prepareScoreCoralL1().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L1));
        controls.prepareScoreCoralL2().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L2));
        controls.prepareScoreCoralL3().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L3));
        controls.prepareScoreCoralL4().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L4));

        controls.algaeAutoAlign()
                .debounce(0.05)
                .whileTrue(Commands.parallel(
                        elevator.setGoalCommand(
                                () -> getClosestReefFace().isAlgaePositionHigh() ? Elevator.Goal.DEALGIFY_HIGH
                                        : Elevator.Goal.DEALGIFY_LOW),
                        Commands.sequence(
                                DriveCommands.alignToAlgaeReef(drive, led, () -> getClosestReefFace(), () -> false)
                                        .until(coralOuttakeRoller.currentSpikeTrigger),
                                DriveCommands.alignToAlgaeReef(drive, led, () -> getClosestReefFace(), () -> true)),
                        Commands.sequence(
                                Commands.waitUntil(() -> elevator.atGoal(Inches.of(2))),
                                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.DEALGIFY),
                                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.DEALGIFY),
                                Commands.waitSeconds(0.2),
                                Commands.waitUntil(coralOuttakeRoller.currentSpikeTrigger),
                                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_HOLD)))
                        .finallyDo(() -> {
                            coralOuttakePivot.setGoal(CoralOuttakePivot.Goal.DEALGIFY_STOW);
                        }))
                .onFalse(
                    Commands.parallel(
                        coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.DEALGIFY_STOW),
                        elevator.setGoalCommand(Elevator.Goal.STOW),
                        coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.ALGAE_HOLD),
                        Commands.sequence(
                            Commands.runOnce(() -> canStartCoralAlign = false),
                            Commands.waitTime(Milliseconds.of(500)),
                            Commands.runOnce(() -> canStartCoralAlign = true)
                        )
                    )
                );

        // controls.algaeAutoAlign()
        // .and(() -> coralMode == CoralMode.L1)
        // .whileTrue(
        // Commands.parallel(
        // elevator.setAlgaeGoalFromCoralMode(() -> coralMode),
        // Commands.sequence(
        // new DriveToPoint(
        // drive,
        // () -> Processor.centerFace.transformBy(DriveCommands.kRobotAlgaeAlignOffset)
        // ),
        // drive.stopCommand()
        // )
        // )
        // )
        // .onFalse(coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.STOW));

        controls.coralAutoAlign()
                .and(() -> coralMode != CoralMode.L1)
                .and(() -> canStartCoralAlign)
                .debounce(0.05)
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.alignToBranchReef(drive, led, () -> getClosestReefFace(),
                                        controls.leftBranchSelectedSupplier(), () -> false),
                                Commands.sequence(
                                        Commands.waitUntil(() -> drive.isWithinToleranceToPose(
                                                DriveCommands.getRobotAlignBranchPoseFromReefFace(
                                                        this::getClosestReefFace,
                                                        controls.leftBranchSelectedSupplier()),
                                                Feet.of(3), Degrees.of(180))),
                                        elevator.setGoalAndWait(() -> Elevator.Goal.fromCoralMode(coralMode)),
                                        coralOuttakePivot
                                                .setGoalAndWait(() -> CoralOuttakePivot.Goal.fromCoralMode(coralMode)),
                                        coralOuttakeRoller.setGoalCommand(
                                                () -> CoralOuttakeRoller.Goal.fromCoralMode(coralMode)))))
                .onFalse(
                        Commands.sequence(
                                coralOuttakePivot.setGoalAndWait(CoralOuttakePivot.Goal.STOW),
                                elevator.setGoalCommand(Elevator.Goal.STOW),
                                coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.STOW)));

        controls.coralAutoAlign().and(() -> coralMode == CoralMode.L1)
            .whileTrue(
                Commands.parallel(
                    DriveCommands.alignToL1Reef(drive, led, this::getClosestReefFace),
                    Commands.sequence(
                        Commands.waitUntil(() -> drive.isWithinToleranceToPose(DriveCommands.getRobotAlignL1FacePoseFromReefFace(this::getClosestReefFace), Feet.of(0.2), Degrees.of(15))),
                        coralIntake.setGoalAndWait(CoralIntake.Goal.L1)
                    )
                )
            ).onFalse(
                coralIntake.setGoalCommand(CoralIntake.Goal.STOW)
            );


        controls.manualShoot().and(controls.coralAutoAlign().or(controls.algaeAutoAlign()))
                .whileTrue(
                        coralOuttakeRoller.setGoalEndCommand(() -> CoralOuttakeRoller.Goal.fromCoralMode(coralMode),
                                CoralOuttakeRoller.Goal.STOW));

        controls.intake().and(controls.coralAutoAlign().or(controls.algaeAutoAlign()).negate())
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> {if (coralMode == CoralMode.L1){
                                                        coralMode = preL1CoralMode;
                                                }
                                                preL1CoralMode = coralMode;
                                        }),
                                Commands.either(coralIntake.setGoalCommand(CoralIntake.Goal.GROUND_INTAKE),
                                                Commands.parallel(coralIntake.setGoalCommand(CoralIntake.Goal.STATION_INTAKE),
                                                                  DriveCommands.alignToStation(drive, led, this::getClosestStation)),
                                        controls.coralIntakeModeSupplier()))
                        )
                .onFalse(indexCoralAndStowCommand());

        controls.intakeL1().and(controls.coralAutoAlign().or(controls.algaeAutoAlign()).negate())
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> {
                                        preL1CoralMode = (coralMode != CoralMode.L1) ? coralMode : preL1CoralMode;
                                        coralMode = CoralMode.L1;
                                }),
                                Commands.either(coralIntake.setGoalCommand(CoralIntake.Goal.GROUND_INTAKE),
                                                Commands.parallel(coralIntake.setGoalCommand(CoralIntake.Goal.STATION_INTAKE),
                                                                  DriveCommands.alignToStation(drive, led, this::getClosestStation)),
                                        controls.coralIntakeModeSupplier())))
                .onFalse(indexCoralAndStowCommand());

        controls.climb()
                .onTrue(
                        Commands.sequence(
                                Commands.parallel(
                                        elevator.setGoalCommand(Elevator.Goal.CLIMB),
                                        coralIntake.setGoalCommand(CoralIntake.Goal.CLIMB),
                                        coralOuttakePivot.setGoalCommand(CoralOuttakePivot.Goal.CLIMB))),
                                DriveCommands.
                        )
                .onFalse(
                        Commands.sequence(
                                coralIntake.setGoalCommand(CoralIntake.Goal.CLIMB_BOTTOM),
                                elevator.setGoalAndWait(Elevator.Goal.CLIMB_BOTTOM).withTimeout(2.2),
                                elevator.setGoalCommand(Elevator.Goal.CLIMB_BOTTOM_LOCK)));

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
                        handoffCommand()
                )
        );

        controls.eject().and(() -> isHandoffInterruptible)
                .onTrue(
                        Commands.parallel(coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_VOMIT, CoralIntake.Goal.STOW),
                                          coralOuttakeRoller.setGoalCommand(CoralOuttakeRoller.Goal.SHOOT_L4))
        );

        controls.reset().whileTrue(new InstantCommand(() -> {
            teleopInit();
        }, coralOuttakePivot, coralOuttakeRoller, coralIntake, elevator));

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
                Commands.waitSeconds(0.1)
        ).finallyDo(() -> {
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

    private ReefFace getClosestReefFace() {
        ReefFace closestFace = null;
        Distance closestDistance = Meters.of(Double.MAX_VALUE);
        Pose2d currentPose = drive.getPose();

        for (ReefFace face : ReefFace.values()) {
            Pose2d rawReefFacePose = FieldConstants.Reef.centerFaces[face.ordinal()];
            Pose2d reefFacePose = AllianceFlipUtil.apply(rawReefFacePose);
            Distance distance = Meters
                    .of(reefFacePose.getTranslation().getDistance(currentPose.getTranslation()));
            if (distance.lt(closestDistance)) {
                closestFace = face;
                closestDistance = distance;
            }
        }

        return closestFace;
    }

    private StationSide getClosestStation() {
        StationSide closestStation = null;
        Distance closestDistance = Meters.of(Double.MAX_VALUE);

        for (StationSide side : StationSide.values()) {
                Pose2d stationPose;

                switch (side) {
                        case RIGHT:
                                stationPose = new Pose2d(Meters.of(1.548), Meters.of(0.767), new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(140)));
                                break;
                        case LEFT:
                                stationPose = new Pose2d(Meters.of(1.336), Meters.of(7.116), new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(220)));
                                break;
                        default:
                                stationPose = null;
                }

                Distance distance = Meters.of(stationPose.getTranslation().getDistance(drive.getPose().getTranslation()));
                if (distance.lt(closestDistance)) {
                        closestDistance = distance;
                        closestStation = side;
                }
        }

        Logger.recordOutput("/closestStation", closestStation);

        return closestStation;
    }

    /** used for logging */
    public void periodic() {
        Logger.recordOutput("FinishedZeroing",
                coralIntake.getZeroSensorDebounced(true) && elevator.getZeroSensorDebounced());
    }
}

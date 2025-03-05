// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.winch.WinchIO;
import frc.robot.subsystems.elevator.winch.WinchIOKraken;
import frc.robot.subsystems.elevator.winch.WinchIOReplay;
import frc.robot.subsystems.elevator.winch.WinchIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import frc.robot.utils.ReefFace;
import frc.robot.utils.RobotViz;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

  private final Controls controls;

  private Superstructure superstructure;

  SendableChooser<Command> m_autoChooser;

  @Getter private final CoralIntake coralIntake;
  @Getter private final CoralOuttake coralOuttake;
  @Getter private final Elevator elevator;
  @Getter private final Drive drive;
  @Getter private final Vision aprilTagVision;

  // @Getter private final CoralCamera coralCamera;

  @Getter private CANBusStatusSignalRegistration elevatorCANBusHandler = new CANBusStatusSignalRegistration();
  @Getter private CANBusStatusSignalRegistration driveCANBusHandler = new CANBusStatusSignalRegistration();

  private Trigger coralIntakeAtStowGoal;
  private Trigger elevatorAtStowGoal;

  private CoralMode coralMode = CoralMode.L4;

  private AddressableLED led = new AddressableLED(2);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(31);

  /** RobotContainer initialization */
  public RobotContainer() {
    SignalLogger.start();

    LEDPattern pattern = LEDPattern.rainbow(255, 128);

    pattern.applyTo(buffer);

    led.setLength(buffer.getLength());

    led.setData(buffer);

    led.start();

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

    VisionIO[] aprilTagVisionIOs;

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
            // Algae Claw
            // wristIO = new WristIOKraken(Ports.AlgaeClaw.wristMotor, Ports.AlgaeClaw.wristEncoder, elevatorCANBusHandler);
            // algaeClawRollerIO = new RollerIOKraken(Ports.AlgaeClaw.algaeClawRoller, elevatorCANBusHandler);

            // Elevator
            winchIO = new WinchIOKraken(Ports.Elevator.LeftWinchMotor,
                                        Ports.Elevator.RightWinchMotor,
                                        Ports.Elevator.WinchEncoder,
                                        driveCANBusHandler);

            // Coral Intake
            beltIO = new RollerIONeo(Ports.CoralIntake.belt, IdleMode.kBrake);
            pivotIO = new PivotIONeo(Ports.CoralIntake.pivotMotor, Ports.CoralIntake.pivotEncoder);
            coralIntakeRollerIO = new RollerIONeo(Ports.CoralIntake.coralIntakeRoller, IdleMode.kCoast);

            // Coral Outtake
            rollerIO = new RollerIOBag(Ports.CoralOuttake.roller);

            // Drive
            gyroIO = new GyroIOPigeon2(driveCANBusHandler);
            flModuleIO = new ModuleIOTalonFX(TunerConstants.FrontLeft, driveCANBusHandler);
            frModuleIO = new ModuleIOTalonFX(TunerConstants.FrontRight, driveCANBusHandler);
            blModuleIO = new ModuleIOTalonFX(TunerConstants.BackLeft, driveCANBusHandler);
            brModuleIO = new ModuleIOTalonFX(TunerConstants.BackRight, driveCANBusHandler);

            drive = new Drive(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);

            aprilTagVisionIOs = new VisionIO[] {
              // new VisionIOPhotonVision("limelight", VisionConstants.kModuleTagRobotToCamera),
              new VisionIOLimelight("limelight-nick", drive::getRotation),
              new VisionIOLimelight("limelight-woody", drive::getRotation)
            };

            aprilTagVision = new Vision(drive::addVisionMeasurement, aprilTagVisionIOs);
            break;
    }

    elevator = new Elevator(winchIO);
    coralIntake = new CoralIntake(beltIO, pivotIO, coralIntakeRollerIO, centerSensor, handoffSensor);
    coralOuttake = new CoralOuttake(rollerIO, handoffSensor);

    superstructure = new Superstructure(coralIntake, elevator, coralOuttake);

    new RobotViz(drive::getPose, coralIntake::getPosition, elevator::getPosition);

    controls = new MatchXboxControls(0, 1);
    configureBindings();

    coralIntakeAtStowGoal = coralIntake.atGoalTrigger(CoralIntake.Goal.STOW, Degrees.of(1.5));
    elevatorAtStowGoal = new Trigger(elevator.atGoalTrigger(Elevator.Goal.STOW));

    // Automatic handoff sequence trigger in teleop
    RobotModeTriggers.teleop()
    .and(coralIntakeAtStowGoal)
    .and(elevatorAtStowGoal)
    .and(coralIntake.centerSensorTrigger)
    .debounce(0.25)
    .onTrue(handoffCommand());

    NamedCommands.registerCommand("ScoreCoralLevel4", Commands.sequence(
      elevator.setGoalAndWait(Elevator.Goal.L4).withTimeout(3.0),
      coralOuttake.setGoalEndCommand(CoralOuttake.Goal.SHOOT, CoralOuttake.Goal.IDLE).withTimeout(0.5),
      elevator.setGoalCommand(Elevator.Goal.STOW)));

    NamedCommands.registerCommand("Handoff", handoffCommand());
    NamedCommands.registerCommand("PrepareLevel4", Commands.none());
    // NamedCommands.registerCommand("PrepareLevel4", elevator.setGoalCommand(Elevator.Goal.L4));
    NamedCommands.registerCommand("PrepareLoadingStationIntake", coralIntake.setGoalCommand(CoralIntake.Goal.STATION_INTAKE));

    NamedCommands.registerCommand("IntakeFromLoadingStation", coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_INTAKE, CoralIntake.Goal.STOW)
    .until(coralIntake.pieceDetectedTrigger.debounce(0.5))
    .withTimeout(2.5));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // coralCamera = new CoralCamera(new VisionIOLimelight("limelight", drive::getPose));
  }

  private double mapInput(double input, double inputMin, double inputMax, double outputMin, double outputMax) {
    return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
  }

  /** Configures bindings to oi */
  private void configureBindings() {
    // Driver

    DoubleSupplier speedMultiplier = () -> {
      if (elevator.getPosition().gt(Inches.of(10))) {
        double elevatorHeight = elevator.getPosition().in(Inches);
        return mapInput(elevatorHeight, 10, 60, 8.0, 0.2);
      }

      return 1.0;
    };

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, controls::getDriveForward, controls::getDriveLeft, controls::getDriveRotation, speedMultiplier));
    // elevator.setDefaultCommand(new InstantCommand(() -> {
    //   if (elevator.getCurrentGoal() != Elevator.Goal.STOW && elevator.getCurrentGoal() != Elevator.Goal.CLIMB_BOTTOM) {
    //     elevator.setGoal(Elevator.Goal.STOW);
    //   }
    // }));
    // coralOuttake.setDefaultCommand(new InstantCommand(() -> {
    //   coralOuttake.setGoal(CoralOuttake.Goal.IDLE);
    // }));
    // coralIntake.setDefaultCommand(new InstantCommand(() -> {
    //   coralIntake.setGoal(CoralIntake.Goal.STOW);
    // }));

    controls.resetDriveHeading().onTrue(drive.resetDriveHeadingCommand());

    controls.driveBrake().onTrue(drive.stopWithXCommand());

    controls.gamePieceLock().whileTrue(DriveCommands.joystickDriveAtAngle(drive, controls::getDriveForward, controls::getDriveLeft, () -> {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ankit");
      double coralOffset = table.getEntry("tx").getDouble(0.0);

      if (Math.abs(coralOffset) != 0.0) {
        coralOffset += 10;
      }

      Logger.recordOutput("AutoIntakeDebug/coralOffset", coralOffset);
      Logger.recordOutput("AutoIntakeDebug/driveRot", drive.getRotation());
      Logger.recordOutput("AutoIntakeDebug/desiredRot", drive.getRotation().getDegrees() + coralOffset);
      // return coralCamera.getCoralOffset().tx().plus(drive.getRotation());

      return drive.getRotation().minus(Rotation2d.fromDegrees(coralOffset));
    }));

    // controls.leftPositionLock().whileTrue(AutoBuilder.7pathfindToPose(new Pose2d(5.27, 3.00, Rotation2d.fromDegrees(120)), new PathConstraints(FeetPerSecond.of(8), FeetPerSecondPerSecond.of(5), DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(480))));

    // controls.groundIntakeCoral().whileTrue(coralIntake.setGoalEndCommand(CoralIntake.Goal.GROUND_INTAKE, CoralIntake.Goal.STOW));

    controls.prepareScoreCoral().and(() -> coralMode == CoralMode.L1).whileTrue(
      coralIntake.setGoalEndCommand(CoralIntake.Goal.L1_Prepare, CoralIntake.Goal.STOW)
    );

    controls.prepareScoreCoral().and(() -> coralMode != CoralMode.L1).whileTrue(
      elevator.setGoalEndCommand(() -> Elevator.Goal.fromCoralMode(coralMode), Elevator.Goal.STOW)
    );

    controls.scoreGamePiece().and(() -> coralMode == CoralMode.L1).whileTrue(
      coralIntake.setGoalEndCommand(CoralIntake.Goal.L1, CoralIntake.Goal.STOW)
    );

    controls.scoreGamePiece().and(() -> coralMode != CoralMode.L1).whileTrue(
      coralOuttake.setGoalEndCommand(CoralOuttake.Goal.SHOOT, CoralOuttake.Goal.IDLE)
    );

    controls.prepareScoreCoralL1().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L1));
    controls.prepareScoreCoralL2().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L2));
    controls.prepareScoreCoralL3().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L3));
    controls.prepareScoreCoralL4().onTrue(Commands.runOnce(() -> coralMode = CoralMode.L4));

    controls.handoffCoral().onTrue(Commands.parallel(
      elevator.setGoalCommand(Elevator.Goal.STOW),
      coralIntake.setGoalCommand(CoralIntake.Goal.STOW),
      coralOuttake.setGoalCommand(CoralOuttake.Goal.IDLE)
    ));

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
          Commands.waitUntil(coralIntake.atGoalTrigger.and(coralIntake.centerSensorTrigger)).withTimeout(0.5),
          coralIntake.setGoalCommand(CoralIntake.Goal.STOW)
        ),
        coralIntake.setGoalCommand(CoralIntake.Goal.STOW),
        coralIntake.handoffSensorTrigger
      )
    );

    controls.groundVomitCoral().whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.GROUND_VOMIT, CoralIntake.Goal.STOW)
    );

    controls.alignToReef().whileTrue(DriveCommands.pathfindToReef(drive,
      () -> ReefFace.fromPOV(controls.getDriverPOV()), controls.alignToReefLeftBranch())
    );

    controls.sourceIntakeCoral().whileTrue(
        coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_INTAKE, CoralIntake.Goal.STOW)
    );

    controls.sourceVomitCoral().whileTrue(
      coralIntake.setGoalEndCommand(CoralIntake.Goal.STATION_VOMIT, CoralIntake.Goal.STOW)
    );

    controls.climb().whileTrue(
      Commands.parallel(
        elevator.setGoalEndCommand(Elevator.Goal.CLIMB, Elevator.Goal.CLIMB_BOTTOM),
        coralIntake.setGoalCommand(CoralIntake.Goal.CLIMB)
      )
    );

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
    coralOuttake.setGoal(CoralOuttake.Goal.IDLE);
  }


  /** Returns the autonomous command */
  public Command getAutonomousCommand() {
    var selected = m_autoChooser.getSelected();

    if (selected == null) {
      return Commands.print("AHHHHHH");
    }

    return selected;
  }

  private Command handoffCommand() {
    return Commands.sequence(
      Commands.parallel(
        elevator.setGoalCommand(Elevator.Goal.STOW),
        coralIntake.setGoalCommand(CoralIntake.Goal.HANDOFF),
        coralOuttake.setGoalCommand(CoralOuttake.Goal.HANDOFF)),
      Commands.waitUntil(coralIntake.handoffSensorTrigger),
      Commands.waitSeconds(0.1),
      Commands.waitUntil(coralOuttake.currentSpikeTrigger).withTimeout(2),
      Commands.waitSeconds(0.2),
      coralIntake.setGoalCommand(CoralIntake.Goal.HANDOFF_PUSH_CORAL_UP),
      elevator.setGoalCommand(Elevator.Goal.HANDOFF),
      Commands.waitSeconds(0.2),
      Commands.waitUntil(coralIntake.handoffSensorTrigger.negate()),
      Commands.parallel(
        coralOuttake.setGoalCommand(CoralOuttake.Goal.CORAL_BACKWARDS),
        coralIntake.setGoalCommand(CoralIntake.Goal.STOW),
            elevator.setGoalCommand(Elevator.Goal.STOW)),
      Commands.waitSeconds(0.3),
        coralOuttake.setGoalCommand(CoralOuttake.Goal.IDLE)
    ).finallyDo(() -> {
      elevator.setGoal(Elevator.Goal.STOW);
      coralIntake.setGoal(CoralIntake.Goal.STOW);
      coralOuttake.setGoal(CoralOuttake.Goal.IDLE);
    });
  }
}

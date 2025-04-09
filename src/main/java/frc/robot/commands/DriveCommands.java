// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.AllianceFlipUtil;
import frc.lib.DriveToPoint;
import frc.lib.LimelightHelpers;
import frc.lib.dashboard.LoggedTunableMeasures.LoggedTunableAngularAcceleration;
import frc.lib.dashboard.LoggedTunableMeasures.LoggedTunableAngularVelocity;
import frc.lib.dashboard.LoggedTunableMeasures.LoggedTunableLinearAcceleration;
import frc.lib.dashboard.LoggedTunableMeasures.LoggedTunableLinearVelocity;
import frc.robot.sensors.VisionConstants;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_outtake_pivot.CoralOuttakePivot;
import frc.robot.subsystems.coral_outtake_roller.CoralOuttakeRoller;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;
import frc.robot.utils.FieldConstants;
import frc.robot.utils.FieldConstants.Barge;
import frc.robot.utils.FieldConstants.CoralStation;
import frc.robot.utils.FieldConstants.Processor;
import frc.robot.utils.ReefFace;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.01;
  private static final double ANGLE_KP = 1.1;
  private static final double ANGLE_KD = 0.5;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier speedMultiplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Apply rotation deadband
          // double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          double omega = omegaSupplier.getAsDouble();

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          double multiplier = speedMultiplier.getAsDouble();

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
              omega * drive.getMaxAngularSpeedRadPerSec() * multiplier);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Robot relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command robotRelativeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(() -> {
      drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(
          new ChassisSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble()),
          drive.getRotation()));
    }, drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for
   * angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target,
   * or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController = new ProfiledPIDController(
        ANGLE_KP,
        0.0,
        ANGLE_KD,
        new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
              ySupplier.getAsDouble());

          // Calculate angular speed
          double omega = angleController.calculate(
              drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega);
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>
   * This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
            () -> {
              drive.runCharacterization(0.0);
            },
            drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runCharacterization(voltage);
              velocitySamples.add(drive.getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                () -> {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  private static final LoggedTunableLinearVelocity kMaxLinearVelocity = new LoggedTunableLinearVelocity(
      "PathfindToReef/MaxLinearVelocity", FeetPerSecond.of(10));
  private static final LoggedTunableLinearAcceleration kMaxLinearAcceleration = new LoggedTunableLinearAcceleration(
      "PathfindToReef/MaxLinearAcceleration", FeetPerSecondPerSecond.of(10));
  private static final LoggedTunableAngularVelocity kMaxAngularVelocity = new LoggedTunableAngularVelocity(
      "PathfindToReef/MaxAngularVelocity", DegreesPerSecond.of(720));
  private static final LoggedTunableAngularAcceleration kMaxAngularAcceleration = new LoggedTunableAngularAcceleration(
      "PathfindToReef/MaxAngularAcceleration", DegreesPerSecondPerSecond.of(720));

  private static final Set<ReefFace> kFlippedReefFaces = EnumSet.of(ReefFace.EF, ReefFace.GH, ReefFace.IJ);

  private static final Transform2d kRobotBranchAlignOffset = new Transform2d(
      new Translation2d(
          Inches.of(20), // F/B
          Inches.of(-1.614) // L/R
      ),
      Rotation2d.k180deg);

  private static final Transform2d kRobotBeforeHandoffBranchAlignOffset = new Transform2d(
    new Translation2d(
        Inches.of(25), // F/B
        Inches.of(-1.614) // L/R
    ),
    Rotation2d.k180deg);


  // TODO idk man just ask someone
  private static final Transform2d kRobotL1Offset = new Transform2d(
    new Translation2d(
      Inches.of(20), // F/B
      Inches.of(-1.614)),
    Rotation2d.kCCW_90deg
  );

  private static final Transform2d kStationOffset = new Transform2d(
    new Translation2d(
      Inches.of(20),
      Inches.of(-1.614)
    ),
    Rotation2d.k180deg
  );

  private static final Transform2d kRobotAlgaeAlignOffset = new Transform2d(
      new Translation2d(
          Inches.of(19.5), // F/B
          Inches.of(-1.614 - 6.5) // L/R
      ),
      Rotation2d.k180deg);

  private static final Transform2d pathPlannerOffset = new Transform2d(
      new Translation2d(Inches.of(33), Inches.of(-1.5)), Rotation2d.k180deg);

  /** Creates a command that drives to a reef position based on POV */
  public static Command pathfindToReef(Drive drive, LED led, Supplier<ReefFace> reefFaceSupplier,
      BooleanSupplier leftBranchSupplier) {
    return Commands.defer(() -> {
      ReefFace face = reefFaceSupplier.get();
      boolean leftBranch = leftBranchSupplier.getAsBoolean();

      if (face == null) {
        return drive.stopWithXCommand();
      }

      PathConstraints constraints = new PathConstraints(kMaxLinearVelocity.get(), kMaxLinearAcceleration.get(),
          kMaxAngularVelocity.get(), kMaxAngularAcceleration.get());
      boolean flipBranchSide = kFlippedReefFaces.contains(face);
      boolean leftSideToDriver = flipBranchSide ^ leftBranch;
      int branchPoseIndex = face.ordinal() * 2 + (leftSideToDriver ? 0 : 1);

      Pose2d pathPlannerTarget = FieldConstants.Reef.branchPositions2d.get(branchPoseIndex)
          .get(FieldConstants.ReefLevel.L1).transformBy(pathPlannerOffset);

      Pose2d target = FieldConstants.Reef.branchPositions2d.get(branchPoseIndex).get(FieldConstants.ReefLevel.L1)
          .transformBy(kRobotBranchAlignOffset);

      Pose2d allianceAppliedTarget = AllianceFlipUtil.apply(target);

      Logger.recordOutput("PathfindToReef/ReefFace", face);
      Logger.recordOutput("PathfindToReef/BranchIndex", branchPoseIndex);
      Logger.recordOutput("PathfindToReef/PPTargetPose", pathPlannerTarget);
      Logger.recordOutput("PathfindToReef/TargetPose", allianceAppliedTarget);

      return Commands.sequence(
          AutoBuilder.pathfindToPose(pathPlannerTarget, constraints),
          new DriveToPoint(drive, () -> allianceAppliedTarget),
          drive.stopWithXCommand());
    }, Set.of(drive));
  }

  private static final List<CoralIntake.Goal> INTAKE_PROCESSOR_GOALS = Arrays.asList(new CoralIntake.Goal[] {
      CoralIntake.Goal.ALGAE_INTAKE, CoralIntake.Goal.ALGAE_SHOOT, CoralIntake.Goal.HOLD_ALGAE });

  private static final List<CoralIntake.Goal> INTAKE_L1_GOALS = Arrays.asList(new CoralIntake.Goal[] {
    CoralIntake.Goal.L1, CoralIntake.Goal.L1_PREPARE
  });

  private static final boolean isNear(Translation2d current, Translation2d target) {
    return current.getDistance(target) < 3;
  }

  /**
   * Creates command that will lock to different field elements based on current
   * robot state and current location
   */
  public static Command fieldElementLock(Drive drive, CoralIntake intake, CoralOuttakeRoller roller,
      CoralOuttakePivot pivot, Elevator elevator, LED led,
      Supplier<ReefFace> reefFaceSupplier, BooleanSupplier leftBumper, BooleanSupplier waitingstate) {
    Supplier<Command> commandSupplier = () -> {
      Translation2d driveTranslation2d = drive.getPose().getTranslation();
      boolean elevatorClimb = elevator.getCurrentGoal() == Elevator.Goal.CLIMB;

      if (INTAKE_PROCESSOR_GOALS.contains(intake.getCurrentGoal())
          && isNear(driveTranslation2d, Processor.centerFace.getTranslation())) {
        Logger.recordOutput("FieldElementLock/CurrentCommand", "alignToProcessor");
        return Commands.sequence(
            new DriveToPoint(drive, () -> Processor.centerFace.transformBy(kRobotAlgaeAlignOffset)),
            // TODO Processor offset probably not same as algae???
            drive.stopCommand());
      }

      if (elevatorClimb) {
        Logger.recordOutput("FieldElementLock/CurrentCommand", "alignToCage");
        return Commands.sequence(
            new DriveToPoint(drive,
                () -> new Pose2d(
                    driveTranslation2d
                        .nearest(List.of(Barge.closeCage, Barge.middleCage, Barge.farCage).stream().map(AllianceFlipUtil::apply).toList()),
                    Rotation2d.k180deg)));
      }
      // TODO AGAIN Station offset probably not same as algae!!!
      if (intake.getCurrentGoal() == CoralIntake.Goal.STATION_INTAKE) {
        if (isNear(driveTranslation2d, CoralStation.leftCenterFace.getTranslation())) {
          Logger.recordOutput("FieldElementLock/CurrentCommand", "alignToCoralStationLeft");
          return Commands.sequence(
              new DriveToPoint(drive, () -> CoralStation.leftCenterFace.rotateAround(CoralStation.leftCenterFace.getTranslation(), Rotation2d.kCCW_90deg).transformBy(kStationOffset)),
              drive.stopCommand());
        } else if (isNear(driveTranslation2d, CoralStation.rightCenterFace.getTranslation())) {
          Logger.recordOutput("FieldElementLock/CurrentCommand", "alignToCoralStationRight");
          return Commands.sequence(
              new DriveToPoint(drive, () -> CoralStation.rightCenterFace.rotateAround(CoralStation.rightCenterFace.getTranslation(), Rotation2d.kCCW_90deg).transformBy(kStationOffset)),
              drive.stopCommand());
        }
      }

      if (INTAKE_L1_GOALS.contains(intake.getCurrentGoal())) {
        Logger.recordOutput("FieldElementLock/CurrentCommand", "alignToL1Reef");
        return alignToL1Reef(drive, led, reefFaceSupplier);
      }

      if (pivot.getCurrentGoal() == CoralOuttakePivot.Goal.DEALGIFY) {
        Logger.recordOutput("FieldElementLock/CurrentCommand", "alignToAlgaeReef");
        return alignToAlgaeReef(drive, led, reefFaceSupplier);
      }

      Logger.recordOutput("FieldElementLock/CurrentCommand", "alignToBranchReef");
      return alignToBranchReef(drive, led, reefFaceSupplier, leftBumper, waitingstate);
    };
    return Commands.defer(commandSupplier, Set.of());
  }

  /** Creates a command that drives to a reef position based on POV */
  public static Command alignToBranchReef(Drive drive, LED led, Supplier<ReefFace> reefFaceSupplier,
      BooleanSupplier leftBranchSupplier, BooleanSupplier waitingState) {
    Supplier<Pose2d> branchPoseSupplier = () -> {
      ReefFace face = reefFaceSupplier.get();
      boolean leftBranch = leftBranchSupplier.getAsBoolean();

      if (face == null) {
        return null;
      }

      // boolean flipBranchSide = kFlippedReefFaces.contains(face);
      // boolean leftSideToDriver = flipBranchSide ^ leftBranch;
      // int branchPoseIndex = face.ordinal() * 2 + (leftSideToDriver ? 0 : 1);

      int branchPoseIndex = face.ordinal() * 2 + (leftBranch ? 0 : 1);

      Transform2d robotTransform2d = waitingState.getAsBoolean() ? kRobotBeforeHandoffBranchAlignOffset : kRobotBranchAlignOffset;

      Pose2d target = FieldConstants.Reef.branchPositions2d.get(branchPoseIndex).get(FieldConstants.ReefLevel.L1)
          .transformBy(robotTransform2d);

      Pose2d allianceAppliedTarget = AllianceFlipUtil.apply(target);

      Logger.recordOutput("PathfindToReef/ReefFace", face);
      Logger.recordOutput("PathfindToReef/BranchIndex", branchPoseIndex);
      Logger.recordOutput("PathfindToReef/TargetPose", allianceAppliedTarget);
      Logger.recordOutput("PathfindToReef/ReefFace", face);
      Logger.recordOutput("PathfindToReef/BranchIndex", branchPoseIndex);
      Logger.recordOutput("PathfindToReef/TargetPose", allianceAppliedTarget);

      return allianceAppliedTarget;
    };

    return Commands.sequence(
        new DriveToPoint(drive, branchPoseSupplier),
        drive.stopCommand());
  }

  /** Creates a command that drives to a branch */
  public static Command alignToBranchReef(Drive drive, LED led, int branchPoseIndex) {
    Supplier<Pose2d> branchPoseSupplier = () -> {
      Pose2d target = FieldConstants.Reef.branchPositions2d.get(branchPoseIndex).get(FieldConstants.ReefLevel.L1)
          .transformBy(kRobotBranchAlignOffset);

      Pose2d allianceAppliedTarget = AllianceFlipUtil.apply(target);

      Logger.recordOutput("PathfindToReef/BranchIndex", branchPoseIndex);
      Logger.recordOutput("PathfindToReef/TargetPose", allianceAppliedTarget);
      Logger.recordOutput("PathfindToReef/BranchIndex", branchPoseIndex);
      Logger.recordOutput("PathfindToReef/TargetPose", allianceAppliedTarget);

      return allianceAppliedTarget;
    };

    return Commands.sequence(
        new DriveToPoint(drive, branchPoseSupplier, Degrees.of(1), Inches.of(1)),
        drive.stopCommand());
  }

  private static final double maxSpeedAutoIntake = 1;

  /**
   * Command to align to a game piece.
   *
   * @param drive                       The {@link Drive} subsystem.
   * @param interpolationFactorSupplier A supplier for the ammount of inerpolation
   *                                    to use. Multipled directly by
   *                                    driverDesired.
   * @param driverDesiredXSupplier      A supplier for the driver desired X value
   *                                    (0.0-1.0).
   * @param driverDesiredYSupplier      A supplier for the driver desired Y value
   *                                    (0.0-1.0).
   * @param coral                       Whether you are aligning to the coral or
   *                                    algae. {@code true} for coral.
   *                                    {@code false} for algae.
   * @return A {@link Command} to align to the game piece.
   */
  public static Command alignToGamePiece(Drive drive, Supplier<Double> interpolationFactorSupplier,
      Supplier<Double> driverDesiredXSupplier, Supplier<Double> driverDesiredYSupplier, Supplier<Boolean> coral) {
    Logger.recordOutput("GRAYCORAL/interpsjefac", interpolationFactorSupplier.get());
    AtomicReference<Pose2d> gamePiecePoseReference = new AtomicReference<>();

    Supplier<Translation2d> getTranslation = () -> {
      // 2d Vector of Driver Desired in Field Space (MPS)
      Translation2d driverDesired = new Translation2d(
        (driverDesiredXSupplier.get() * kMaxLinearVelocity.get().in(MetersPerSecond)),
        (driverDesiredYSupplier.get() * kMaxLinearVelocity.get().in(MetersPerSecond)));

      // 2d Pose of Piece in FieldSpace
      Pose2d piecePose = getPiecePose(drive.getPose(), gamePiecePoseReference, coral.get());
      // Pose2d piecePose = new Pose2d(2, 2, new Rotation2d());
      Logger.recordOutput("GRAYCORAL/piecePose", piecePose);

      if (piecePose == null)
        return driverDesired;

      Translation2d poseError = piecePose.getTranslation().minus(drive.getPose().getTranslation());
      Logger.recordOutput("GRAYCORAL/poseError", poseError);

      poseError = new Translation2d(
          MathUtil.clamp(poseError.getNorm(), 0, kMaxLinearVelocity.get().in(MetersPerSecond)), poseError.getAngle());

      if (Math.abs(poseError.getAngle().minus(driverDesired.getAngle()).getDegrees()) < 20) {
        Translation2d driverMagnitudeButPoseErrorDirectionVector = new Translation2d(driverDesired.getNorm(),
            poseError.getAngle());
        return driverMagnitudeButPoseErrorDirectionVector.interpolate(poseError, interpolationFactorSupplier.get() * interpolationFactorSupplier.get());
      }

      return driverDesired.interpolate(poseError, interpolationFactorSupplier.get() * interpolationFactorSupplier.get());
    };

    Logger.recordOutput("GRAYCORAL/FINALS_TRANSLATION", getTranslation.get());

    return joystickDriveAtAngle(drive, () -> getTranslation.get().getX(), () -> getTranslation.get().getY(),
        () -> getPieceRotationError(gamePiecePoseReference, drive.getPose(), coral.get()));
  }

  /**
   * Creates a command that drives to reef position, aligned to the center of the
   * face.
   */
  public static Command alignToAlgaeReef(Drive drive, LED led, Supplier<ReefFace> reefFaceSupplier) {
    Supplier<Pose2d> branchPoseSupplier = () -> {
      ReefFace face = reefFaceSupplier.get();

      if (face == null) {
        return null;
      }

      Pose2d target = FieldConstants.Reef.branchPositions2d.get(face.ordinal() * 2 + 1).get(FieldConstants.ReefLevel.L1)
          .transformBy(kRobotAlgaeAlignOffset);

      Pose2d allianceAppliedTarget = AllianceFlipUtil.apply(target);

      Logger.recordOutput("PathfindToReefALGAE/ReefFace", face);
      Logger.recordOutput("PathfindToReefALGAE/TargetPose", allianceAppliedTarget);

      return allianceAppliedTarget;
    };

    return Commands.sequence(
        new DriveToPoint(drive, branchPoseSupplier),
        drive.stopCommand());
  }

  /**
   * Creates a command that drives to reef position, aligned to the center of the
   * face.
   */

  public static Command alignToL1Reef(Drive drive, LED led, Supplier<ReefFace> reefFaceSupplier) {
    Supplier<Pose2d> branchPoseSupplier = () -> {
      ReefFace face = reefFaceSupplier.get();

      if (face == null) {
        return null;
      }


      // TODO Might have to be 90 CCW?
      Pose2d target = FieldConstants.Reef.branchPositions2d.get(face.ordinal() * 2 + 1).get(FieldConstants.ReefLevel.L1)
          .transformBy(kRobotL1Offset);

      Pose2d allianceAppliedTarget = AllianceFlipUtil.apply(target);

      Logger.recordOutput("PathfindToReefALGAE/ReefFace", face);
      Logger.recordOutput("PathfindToReefALGAE/TargetPose", allianceAppliedTarget);

      return allianceAppliedTarget;
    };

    return Commands.sequence(
        new DriveToPoint(drive, branchPoseSupplier),
        drive.stopCommand());
  }

  /** Returns the piece pose in Field Space (Cached if necessary) */
  private static Pose2d getPiecePose(Pose2d robotPose, AtomicReference<Pose2d> gamePiecePoseReference, boolean coral) {
    // "d = (h2-h1) / tan(a1+a2)"
    Pose3d robotPose3d = new Pose3d(robotPose);

    Pose3d cameraPose3d = robotPose3d.transformBy(VisionConstants.kIntakeClassifierRobotToCamera);

    Distance cameraHeight = cameraPose3d.getTranslation().getMeasureZ();
    Angle cameraAngle = VisionConstants.kIntakeClassifierRobotToCamera.getRotation().getMeasureY().times(-1);
    Angle cameraXAngle = Degrees.of(90).minus(VisionConstants.kIntakeClassifierRobotToCamera.getRotation().getMeasureZ());
    // cameraXAngle = Degrees.zero();

    Distance targetHeight = Inches.of(2.25);

    Angle targetY = Degrees.of(LimelightHelpers.getTY(VisionConstants.kIntakeClassifierCameraName));
    Angle targetX = Degrees.of(LimelightHelpers.getTX(VisionConstants.kIntakeClassifierCameraName));

    if (targetX.isEquivalent(Degrees.zero())) {
      return gamePiecePoseReference.get();
    }

    double tanVertical = Math.tan(cameraAngle.plus(targetY).times(-1).in(Radians));
    Distance distanceX = (cameraHeight.minus(targetHeight)).div(tanVertical);
    Distance distanceDiag = Meters.of(Math.hypot(distanceX.in(Meters), cameraHeight.in(Meters)));

    double tanHorizontal = Math.tan(cameraXAngle.plus(targetX).in(Radians));

    Distance distanceY = distanceDiag.times(-tanHorizontal);
    // distanceX = distanceX.times(-1);

    Transform2d cameraToTarget = new Transform2d(distanceX.times(0.891), distanceY, Rotation2d.kZero);

    Pose2d targetPose = cameraPose3d.toPose2d().transformBy(cameraToTarget);

    if (gamePiecePoseReference.get() != null) {
      targetPose = targetPose.interpolate(gamePiecePoseReference.get(), 0.5);
    }

    gamePiecePoseReference.set(targetPose);

    Logger.recordOutput("AlignToGamePiece/DistanceX", distanceX);
    Logger.recordOutput("AlignToGamePiece/DistanceY", distanceY);
    Logger.recordOutput("AlignToGamePiece/TargetPose", targetPose);
    Logger.recordOutput("AlignToGamePiece/CameraPose", cameraPose3d);
    Logger.recordOutput("AlignToGamePiece/CameraAngle", cameraAngle.in(Degrees));
    Logger.recordOutput("AlignToGamePiece/CameraXAngle", cameraXAngle.in(Degrees));


    return gamePiecePoseReference.get();
  };

  private static Rotation2d getPieceRotationError(AtomicReference<Pose2d> gamePiecePoseReference, Pose2d robotPose,
      boolean coral) {
    Rotation2d offset = Rotation2d.fromDegrees(-77);
    Pose2d targetPose = getPiecePose(robotPose, gamePiecePoseReference, coral);

    if (targetPose == null) {
      return robotPose.getRotation();
    }

    return targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle().plus(offset);
  }
}

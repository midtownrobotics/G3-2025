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

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.sensors.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.vision.VisionIO;
import frc.robot.sensors.vision.VisionIO.PoseObservation;
import frc.robot.sensors.vision.VisionIO.PoseObservationType;
import frc.robot.sensors.vision.VisionIO.VisionIOInputs;
import frc.robot.sensors.vision.VisionIOInputsAutoLogged;
import frc.robot.utils.LoggerUtil;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Consumer<Pose2d> resetPoseConsumer;

  // public static void main(String[] args) {
  //   Pose3d desiredPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(19).get().transformBy(new Transform3d(new Translation3d(Inches.of(37).div(2), Inches.of(4.5), Inches.of(-12.13)), Rotation3d.kZero));
  //   // X: 3.75m, Y: 5.09m, Z: -0.02m, Roll: -2.63°, Pitch: 0.98°, Yaw: -59.12°
  //   Pose3d observedPose = new Pose3d(3.75, 5.09, -0.02, new Rotation3d(Degrees.of(-2.63), Degrees.of(0.98), Degrees.of(-59.12)));

  //   System.out.println(observedPose.minus(desiredPose));
  // }

  /**
   * Creates a new Vision subsystem.
   */
  public Vision(VisionConsumer consumer, Consumer<Pose2d> resetPoseConsumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.resetPoseConsumer = resetPoseConsumer;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert(
          "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing
   * with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera/" + io[i].getName(), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      String name = io[cameraIndex].getName();
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {

        Pose3d pose = observation.pose();

        // Check whether to reject pose
        boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
            || (observation.type() == VisionIO.PoseObservationType.MEGATAG_2) //&&RobotState.isDisabled())
            || (observation.tagCount() == 1
                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
            || Math.abs(pose.getZ()) > maxZError // Must have realistic Z coordinate

            || observation.averageTagDistance() > Units.feetToMeters(12)
            // Must be within the field boundaries
            || pose.getX() <= 0.0
            || pose.getX() > aprilTagLayout.getFieldLength()
            || pose.getY() <= 0.0
            || pose.getY() > aprilTagLayout.getFieldWidth();
        // || angleDeltaTooGreat(observation, inputs[cameraIndex]);

        // Add pose to log
        robotPoses.add(pose);
        if (rejectPose) {
          robotPosesRejected.add(pose);
        } else {
          robotPosesAccepted.add(pose);
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.averageTagDistance() > 1.0) {
          angularStdDev *= 1.5;
        }
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        if (cameraIndex == 0) {
          Logger.recordOutput("Vision/Camera/" + name + "/preRotationLog", pose.toPose2d());
          pose = pose.rotateAround(pose.getTranslation(), new Rotation3d(0, 0, Math.PI));
          Logger.recordOutput("Vision/Camera/" + name + "/postRotationLog", pose.toPose2d());
          Logger.recordOutput("SEENPLEASE", Logger.getTimestamp());
        }

        Logger.recordOutput("Vision/Camera/" + name + "/rightBeforeAcception", pose.toPose2d());

        // Send vision observation
        consumer.accept(
            pose.toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        if (RobotState.isDisabled() && observation.type() == VisionIO.PoseObservationType.MEGATAG_1) {
          resetPoseConsumer.accept(pose.toPose2d());
          Logger.recordOutput("ThisThingWasLastSeenAt", Logger.getTimestamp());
        }
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera/" + name + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera/" + name + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera/" + name + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera/" + name + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    /**
     * Accepts a vision measurement to update the robot pose estimate.
     */
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /** Command to enable or disable a specific camera */
  public Command enableDisableCamera(boolean enabled, int cameraIndex) {
    return run(() -> io[cameraIndex].setEnabled(enabled));
  };

  private boolean angleDeltaTooGreat(PoseObservation observation, VisionIOInputs inputs) {
    return observation.pose().transformBy(inputs.transformRobotToCamera)
        .minus(aprilTagLayout.getTagPose(inputs.tagIds[0]).get())
        .getRotation().getMeasureAngle().abs(Degrees) > 70;
  }
}

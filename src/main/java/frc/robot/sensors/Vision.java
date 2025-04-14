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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.vision.VisionIO;
import frc.robot.sensors.vision.VisionIO.PoseObservation;
import frc.robot.sensors.vision.VisionIO.PoseObservationType;
import frc.robot.sensors.vision.VisionIO.VisionIOInputs;
import frc.robot.sensors.vision.VisionIOInputsAutoLogged;
import java.util.Arrays;
import frc.robot.utils.LoggerUtil;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  /**
   * Creates a new Vision subsystem.
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

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

      List<PoseObservation> poseObservationList = Arrays.asList(inputs[cameraIndex].poseObservations);

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {

        // Add pose to log
        robotPoses.add(observation.pose);
        
//         if (angleDeltaTooGreat(observation, inputs[cameraIndex])) {
//           robotPosesRejected.add(observation.pose);
//           continue;
//         }

        if (observation.tagCount == 0) {
          robotPosesRejected.add(observation.pose);
          continue;
        }

        if (observation.tagCount == 1 && observation.ambiguity > maxAmbiguity) {
          robotPosesRejected.add(observation.pose);
          continue;
        }

        if (Math.abs(observation.pose.getZ()) > maxZError) {
          robotPosesRejected.add(observation.pose);
          continue;
        }

        if (observation.averageTagDistance > Units.feetToMeters(10)) {
          robotPosesRejected.add(observation.pose);
          continue;
        }

        if (observation.pose.getX() <= 0.0 || observation.pose.getX() > aprilTagLayout.getFieldLength()
            || observation.pose.getY() <= 0.0 || observation.pose.getY() > aprilTagLayout.getFieldWidth()) {
          robotPosesRejected.add(observation.pose);
          continue;
        }

        if (observation.type == PoseObservationType.MEGATAG_2 && poseObservationList.stream()
            .anyMatch((o) -> o.type == PoseObservationType.MEGATAG_1 && o.pose.toPose2d().getRotation()
                .minus(observation.pose.toPose2d().getRotation()).getMeasure().gt(Degrees.of(10)))) {
          robotPosesRejected.add(observation.pose);
          continue;
        }

        robotPosesAccepted.add(observation.pose);

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.averageTagDistance, 3.0) / observation.tagCount;
        
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.averageTagDistance > 1.0) {
          angularStdDev *= 2.4;
        }
        if (observation.type == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose.toPose2d(),
            observation.timestamp,
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
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

  private boolean angleDeltaTooGreat (PoseObservation observation, VisionIOInputs inputs) {
    return observation.pose().transformBy(inputs.transformRobotToCamera)
  .minus(aprilTagLayout.getTagPose(inputs.tagIds[0]).get())
  .getRotation().getMeasureAngle().abs(Degrees) > 70;
  }
}

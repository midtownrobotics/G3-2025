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

package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * VisionIO provides an interface for vision systems to update and provide
 * vision data
 * for the robot, including pose and target observations. This is typically
 * implemented
 * by specific vision hardware or libraries (e.g., Limelight, PhotonVision,
 * etc.).
 */
public interface VisionIO {

  /** get enabled */
  public boolean getEnabled();

  /** set enabled */
  public void setEnabled(boolean enabled);

  /**
   * Class to store inputs related to vision data, including connection status,
   * target observations, pose observations, and tag IDs.
   */
  @AutoLog
  public static class VisionIOInputs {
    /**
     * Indicates whether the vision system is connected.
     */
    public boolean connected = false;

    /**
     * Indicates whether the vision system detects a target
     */
    public boolean targetSeen = false;

    /**
     * The latest target observation, including the horizontal and vertical offsets.
     */
    public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());

    /**
     * A list of pose observations for the robot based on the vision system's data.
     */
    public PoseObservation[] poseObservations = new PoseObservation[0];

    /**
     * A list of pose observations estimated by MegaTag2 for the robot
     */
    public PoseObservation[] poseObservationsMegaTag2 = new PoseObservation[0];

    /**
     * A list of tag IDs detected by the vision system.
     */
    public int[] tagIds = new int[0];
  }

  /**
   * Represents the angle to a simple target, not used for pose estimation.
   * This data contains the horizontal and vertical offsets to the target.
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
  }

  /**
   * Represents a robot pose sample used for pose estimation. This includes
   * the timestamp of the observation, the robot's 3D pose, the ambiguity of the
   * observation,
   * the number of tags detected, the average tag distance, and the type of the
   * observation.
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {
  }

  /**
   * Enum to represent different types of pose observations.
   */
  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  /**
   * Gets the name of the vision system.
   */
  public String getName();

  /**
   * Updates the provided inputs object with the latest vision data from the
   * vision system.
   * This method typically gets called to refresh the latest target and pose
   * information.
   *
   * @param inputs The VisionIOInputs object to update with the latest vision
   *               data.
   */
  public void updateInputs(VisionIOInputs inputs);

  /**
   * Set the pipeline. Only for limelights.
   */
  public void setPipeline(long pipelineID);

  /**
   * Returns the pose estimation in single tag mode based on trig
   */
  public PoseObservation trigPoseEstimation();
}

package frc.robot.sensors;

import frc.lib.team1648.Limelight;

/**
 * This class represents the front-facing Limelight camera used for vision processing
 * in a robotics application. It can switch between different vision pipelines
 * for various tasks such as detecting AprilTags, algae, and reef branches.
 */
public class FrontLimelight {

  /**
   * Enum representing the different vision processing pipelines.
   */
  public enum Pipeline {
    APRILTAG, // Pipeline for detecting AprilTags
    ALGAE,    // Pipeline for detecting algae
    REEF;     // Pipeline for detecting reef branches
  }

  private Pipeline currentPipeline;
  private Limelight limelight;

  /**
   * Gets the vision observation from the Limelight if the current pipeline
   * is set to APRILTAG and the target is seen.
   *
   * @return VisionObservation object if a target is seen and the pipeline
   *         is APRILTAG, otherwise null.
   */
  public VisionObservation getVisionObservation() {
    if (currentPipeline != Pipeline.APRILTAG || !limelight.isTargetSeen()) {
      return null;
    }
    return limelight.getBotPoseEstimate();
  }

  /**
   * Gets the horizontal offset of the detected algae if the current pipeline
   * is set to ALGAE and the target is seen.
   *
   * @return Double value representing the horizontal offset if the target is seen
   *         and the pipeline is ALGAE, otherwise null.
   */
  public Double getAlgaeOffset() {
    if (currentPipeline != Pipeline.ALGAE || !limelight.isTargetSeen()) {
      return null;
    }
    return limelight.getHorizontalOffset();
  }

  /**
   * Gets the horizontal offset of the detected reef branch if the current pipeline
   * is set to REEF and the target is seen.
   *
   * @return Double value representing the horizontal offset if the target is seen
   *         and the pipeline is REEF, otherwise null.
   */

  public Double getReefBranchOffset() {
    if (currentPipeline != Pipeline.REEF || !limelight.isTargetSeen()) {
      return null;
    }
    return limelight.getHorizontalOffset();
  }

  /**
   * Sets the current vision processing pipeline to the specified new pipeline.
   *
   * @param newPipeline The new pipeline to set.
   */
  public void setPipeline(Pipeline newPipeline) {
    currentPipeline = newPipeline;
  }

  /**
   * Gets the current vision processing pipeline.
   *
   * @return The current pipeline.
   */
  public Pipeline getPipeline() {
    return currentPipeline;
  }

}

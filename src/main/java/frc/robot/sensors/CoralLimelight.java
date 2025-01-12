package frc.robot.sensors;

import frc.lib.team1648.Limelight;

/**
 * The CoralLimelight class interacts with the Limelight camera, specifically for controlling and
 * reading data from the camera while using different pipelines such as Coral and AprilTag.
 */
public class CoralLimelight {

  private Limelight limelight;

  /** Enum representing the different pipelines that can be used by the Limelight camera. */
  public enum Pipeline {
    CORAL, // Pipeline used for Coral processing.
    APRILTAG; // Pipeline used for AprilTag processing.
  }

  private Pipeline currentPipeline;

  /**
   * Retrieves the horizontal offset for the Coral pipeline. If the current pipeline is set to
   * AprilTag or no target is seen, it returns {@code null}.
   *
   * @return The horizontal offset (in degrees) of the target relative to the Limelight, or {@code
   *     null} if no target is seen or if the pipeline is set to AprilTag.
   */
  public Integer getCoralOffset() {
    // Check if the current pipeline is AprilTag or no target is seen
    if (currentPipeline == Pipeline.APRILTAG || !limelight.isTargetSeen()) {
      return null; // Return null if conditions are not met
    }

    // Return the horizontal offset in degrees
    return (int) limelight.getHorizontalOffset();
  }

  /**
   * Sets the pipeline for the Limelight camera.
   *
   * @param newPipeline The pipeline to switch to (either Coral or AprilTag).
   */
  public void setPipeline(Pipeline newPipeline) {
    currentPipeline = newPipeline;
  }

  /**
   * Gets the current pipeline being used by the Limelight camera.
   *
   * @return The current {@link Pipeline} (either Coral or AprilTag).
   */
  public Pipeline getPipeline() {
    return currentPipeline;
  }
}

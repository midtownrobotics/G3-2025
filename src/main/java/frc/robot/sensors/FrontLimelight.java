package frc.robot.sensors;

import frc.lib.team1648.Limelight;

public class FrontLimelight {
  // getReefBranch()

  private Limelight limelight;

  /** Enum representing the different pipelines that can be used by the Limelight camera. */
  public enum Pipeline {
    ALGAE(0), // Pipeline used for Algae processing.
    REEF(0), // Pipeline used for Reef processing.
    APRILTAG(0); // Pipeline used for AprilTag processing.

    private long pipelineID;

    Pipeline (long ID) {
      this.pipelineID = ID;
    }

    public long getPipelineID() {
      return pipelineID;
    }
  }

  private Pipeline currentPipeline;

  /**
   * Constructs the Limelight for the front of the robot
   */
  public FrontLimelight(String name) {
    limelight = new Limelight(name);

    currentPipeline = Pipeline.APRILTAG;
  }

  /**
   * Retrieves the horizontal offset for the Algae pipeline. If the current pipeline is not set to
   * Algae or no target is seen, it returns {@code null}.
   *
   * @return The horizontal offset (in degrees) of the target relative to the Limelight, or {@code
   *     null} if no target is seen or if the pipeline is set to AprilTag.
   */
  public Integer getAlgaeOffset() {
    // Check if the current pipeline is AprilTag or no target is seen
    if (currentPipeline != Pipeline.ALGAE || !limelight.isTargetSeen()) {
      return null; // Return null if conditions are not met
    }

    // Return the horizontal offset in degrees
    return (int) limelight.getHorizontalOffset();
  }

    /**
   * Retrieves the horizontal offset for the Algae pipeline. If the current pipeline is not set to
   * Algae or no target is seen, it returns {@code null}.
   *
   * @return The horizontal offset (in degrees) of the target relative to the Limelight, or {@code
   *     null} if no target is seen or if the pipeline is set to AprilTag.
   */
  public Integer getReefOffset() {
    // Check if the current pipeline is AprilTag or no target is seen
    if (currentPipeline != Pipeline.REEF || !limelight.isTargetSeen()) {
      return null; // Return null if conditions are not met
    }

    // Return the horizontal offset in degrees
    return (int) limelight.getHorizontalOffset();
  }

  /**
   * Retrieves the vision observation from the Limelight camera, which includes the bot's pose
   * estimate and other vision-related data.
   *
   * @return A {@link VisionObservation} containing the bot's pose estimate and related data or
   * {@code null} if pipeline is incorrect.
   */
  public VisionObservation getVisionObservation() {
    if (currentPipeline != Pipeline.APRILTAG) {
      return null; // Return null if conditions are not met
    }

    return limelight.getBotPoseEstimate();
  }

  /**
   * Sets the pipeline for the Limelight camera.
   *
   * @param newPipeline The pipeline to switch to (either Coral or AprilTag).
   */
  public void setPipeline(Pipeline newPipeline) {
    currentPipeline = newPipeline;

    limelight.setPipeline(newPipeline.getPipelineID());
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

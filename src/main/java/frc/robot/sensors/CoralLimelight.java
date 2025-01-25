package frc.robot.sensors;


import frc.lib.team1648.Limelight;
import frc.robot.subsystems.coral_outtake.CoralOuttakeConstants;

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
  public Double getCoralOffset() {
    // Check if the current pipeline is AprilTag or no target is seen
    if (currentPipeline != Pipeline.CORAL || !limelight.isTargetSeen()) {
      return null; // Return null if conditions are not met
    }

    // Return the horizontal offset in degrees
    return limelight.getHorizontalOffset();
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

  /**
   * Estimates if the piece is being intaked head on or on the side. Returns null if not in the correct mode.
   * @return
   */
  public Boolean isHeadOn() {
    // Check if the current pipeline is AprilTag or no target is seen
    if (currentPipeline != Pipeline.CORAL || !limelight.isTargetSeen()) {
      return null; // Return null if conditions are not met
    }

    // real logic time

    // find the longest side
    boolean longestSideIs0To1;

    double[][] corners = limelight.getCorners();

    longestSideIs0To1 = Math.hypot(
        corners[0][0]-corners[1][0],
        corners[0][1]-corners[1][1]
      ) > Math.hypot(
        corners[1][0]-corners[2][0],
        corners[1][1]-corners[2][1]
      );

    double angle = limelight.getTargetBoxSkewDegrees();

    if (longestSideIs0To1 && Math.abs(angle+0) < CoralOuttakeConstants.HEAD_ON_ANGLE_ERROR) {
      return true;
    } else if (!longestSideIs0To1 && Math.abs(angle+90) < CoralOuttakeConstants.HEAD_ON_ANGLE_ERROR) {
      return true;
    } else {
      return false;
    }

  }


}

package frc.robot.sensors;

import frc.lib.team1648.Limelight;

/**
 * The BackLimelight class provides an interface to interact with the Limelight camera for vision
 * processing. It wraps the Limelight class to obtain the bot's pose estimate and vision
 * observations from the camera.
 */
public class BackLimelight {

  private Limelight limelight;

  /**
   * Constructs a BackLimelight instance with the given Limelight camera name.
   *
   * @param name The name of the Limelight camera to interact with.
   */
  public BackLimelight(String name) {
    limelight = new Limelight(name);
  }

  /**
   * Retrieves the vision observation from the Limelight camera, which includes the bot's pose
   * estimate and other vision-related data.
   *
   * @return A {@link VisionObservation} containing the bot's pose estimate and related data.
   */
  public VisionObservation getVisionObservation() {
    return limelight.getBotPoseEstimate();
  }
}

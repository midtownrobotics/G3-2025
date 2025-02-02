package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.VisionObservation;
import frc.robot.sensors.vision.VisionIO.PoseObservation;

public class AprilTagCamera extends SubsystemBase {
  private VisionIO visionController;
  private VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  /**
   * Constructs the AprilTag Camera
   */
  public AprilTagCamera(VisionIO visionController) {
    this.visionController = visionController;
  }

  @Override
  public void periodic() {
    visionController.updateInputs(visionInputs);
  }

  /**
   * Retrieves the vision observation from the Vision controller, which includes the bot's pose
   * estimate and other vision-related data.
   *
   * @return A {@link VisionObservation} containing the bot's pose estimate and related data
   * or {@code null} if no vision observation was found.
   */
  public PoseObservation getVisionObservation() {
    if (visionInputs.poseObservations.length > 0) {
        return visionInputs.poseObservations[visionInputs.poseObservations.length-1];
    } else {
        return null;
    }
  }
}

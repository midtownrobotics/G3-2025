package frc.robot.sensors;

import frc.lib.VirtualSubsystem;
import frc.robot.sensors.vision.VisionIO;
import frc.robot.sensors.vision.VisionIO.PoseObservation;
import frc.robot.sensors.vision.VisionIOInputsAutoLogged;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AprilTagCamera extends VirtualSubsystem {
  @Getter private final VisionIO visionController;
  private VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();



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

package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.VisionObservation;
import frc.robot.sensors.vision.VisionIO.PoseObservation;
import frc.robot.sensors.vision.VisionIO.TargetObservation;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AlgaeCamera extends SubsystemBase {
    private final VisionIO visionController;
    private VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    /** Enum representing the different pipelines that can be used by the camera controller. */
    @RequiredArgsConstructor
    public enum Pipeline {
        APRILTAG_MT1(0), // Pipeline used for AprilTag processing.
        APRILTAG_MT2(0), // Pipeline used for AprilTag processing.
        ALGAE(1); // Pipeline used for Algae processing.

        @Getter private final int pipelineID;
    }

    @Getter private Pipeline currentPipeline = Pipeline.APRILTAG_MT1;

    @Override
    public void periodic() {
        visionController.updateInputs(visionInputs);
    }

    /**
     * Retrieves the vision observation from the Vision controller, which includes the bot's pose
     * estimate and other vision-related data.
     *
     * @return A {@link VisionObservation} containing the bot's pose estimate and related data or
     * {@code null} if no vision observation was found or the current pipeline is not set to APRILTAG.
     */
    public PoseObservation getVisionObservation() {
        if (currentPipeline == Pipeline.APRILTAG_MT1 && visionInputs.poseObservations.length > 0) {
            return visionInputs.poseObservations[visionInputs.poseObservations.length-1];
        }
        return null;
    }

    /**
     * Retrieves the rotational algae offset.
     *
     * @return A {@link Rotation2d} of the algae offset or {@code null} if the current pipeline is not
     * set to ALGAE.
     */
    public TargetObservation getAlgaeOffset() {
        if (currentPipeline == Pipeline.ALGAE) {
            return visionInputs.latestTargetObservation;
        }
        return null;
    }
}

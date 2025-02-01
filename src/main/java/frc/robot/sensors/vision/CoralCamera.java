package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.VisionObservation;
import frc.robot.sensors.vision.VisionIO.PoseObservation;
import lombok.Getter;

public class CoralCamera extends SubsystemBase {
    private VisionIO visionController;
    private VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    /** Enum representing the different pipelines that can be used by the camera controller. */
    public enum Pipeline {
        CORAL(0), // Pipeline used for Algae processing.
        APRILTAG(0); // Pipeline used for AprilTag processing.

        private long pipelineID;

        Pipeline (long ID) {
            this.pipelineID = ID;
        }

        public long getPipelineID() {
            return pipelineID;
        }
    }

    @Getter Pipeline currentPipeline = Pipeline.APRILTAG;

    /**
     * Constructs the AprilTag Camera
     */
    public CoralCamera(VisionIO visionController) {
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
     * @return A {@link VisionObservation} containing the bot's pose estimate and related data or
     * {@code null} if no vision observation was found or the current pipeline is not set to APRILTAG.
     */
    public PoseObservation getVisionObservation() {
        if (visionInputs.poseObservations.length > 0 && currentPipeline == Pipeline.APRILTAG) {
            return visionInputs.poseObservations[visionInputs.poseObservations.length-1];
        }
        return null;
    }

    /**
     * Retrieves the rotational coral offset.
     *
     * @return A {@link Rotation2d} of the coral offset or {@code null} if the current pipeline is not
     * set to CORAL.
     */
    public Rotation2d getCoralOffset() {
        if (currentPipeline == Pipeline.CORAL) {
            return visionInputs.latestTargetObservation.tx();
        }
        return null;
    }
}

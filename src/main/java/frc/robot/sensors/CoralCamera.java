package frc.robot.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sensors.vision.VisionIO;
import frc.robot.sensors.vision.VisionIO.PoseObservation;
import frc.robot.sensors.vision.VisionIO.TargetObservation;
import frc.robot.sensors.vision.VisionIOInputsAutoLogged;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class CoralCamera extends AprilTagCamera {
    private VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();
    private final Time expirationDuration = Units.Milliseconds.of(200);
    private Time lastObservationTimestamp;
    private TargetObservation lastObservation;

    /**
     * Enum representing the different pipelines that can be used by the camera
     * controller.
     */

    @RequiredArgsConstructor
    public enum Pipeline {
        APRILTAG_MT1(0), // Pipeline used for AprilTag processing.
        APRILTAG_MT2(0), // Pipeline used for AprilTag processing.
        CORAL(1); // Pipeline used for Algae processing.

        @Getter
        private final int pipelineID;
    }

    @Getter
    private Pipeline currentPipeline = Pipeline.CORAL;

    /**
     * Constructor for CoralCamera
     *
     * @param visionController the visionIO object used by the camera
     */
    public CoralCamera(VisionIO visionController) {
        super(visionController);
    }

    @Override
    public void periodic() {
        super.getVisionController().updateInputs(visionInputs);
        if (visionInputs.targetSeen) {
            lastObservation = visionInputs.latestTargetObservation;
            lastObservationTimestamp = Units.Seconds.of(Timer.getFPGATimestamp());
        }
    }

    /**
     * Retrieves the vision observation from the Vision controller, which includes
     * the bot's pose
     * estimate and other vision-related data.
     *
     * @return A {@link VisionObservation} containing the bot's pose estimate and
     *         related data or
     *         {@code null} if no vision observation was found or the current
     *         pipeline is not set to APRILTAG.
     */
    public PoseObservation getVisionObservation() {
        if (currentPipeline != Pipeline.APRILTAG_MT1 && currentPipeline != Pipeline.APRILTAG_MT2) {
            return null;
        }
        if (visionInputs.poseObservations.length == 0 && visionInputs.poseObservationsMegaTag2.length == 0) {
            return null;
        }
        if (currentPipeline == Pipeline.APRILTAG_MT1) {
            return visionInputs.poseObservations[visionInputs.poseObservations.length - 1];
        }
        if (currentPipeline == Pipeline.APRILTAG_MT2) {
            return visionInputs.poseObservationsMegaTag2[visionInputs.poseObservationsMegaTag2.length - 1];
        }
        return null;
    }

    /**
     * Retrieves the rotational coral offset.
     *
     * @return A {@link Rotation2d} of the coral offset or {@code null} if the
     *         current pipeline is not
     *         set to CORAL.
     */
    public TargetObservation getCoralOffset() {
        if (
            Units.Seconds.of(Timer.getFPGATimestamp()).minus(lastObservationTimestamp).lte(expirationDuration)
            && currentPipeline == Pipeline.CORAL
        ) {
            return lastObservation;
        }
        return null;
    }
}

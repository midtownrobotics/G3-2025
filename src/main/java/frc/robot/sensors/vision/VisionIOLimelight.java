package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team1648.Limelight;
import frc.robot.sensors.VisionObservation;

/**
 * VisionIO implementation for the Limelight camera.
 * This class interfaces with the Limelight camera to provide vision data for the robot.
 */
public class VisionIOLimelight implements VisionIO {

    private Limelight limelight;

    /**
     * Constructs a VisionIOLimelight object.
     *
     * @param name The name of the Limelight camera to interface with.
     */
    public VisionIOLimelight(String name) {
        limelight = new Limelight(name);
    }

    /**
     * Updates the inputs with the latest data from the Limelight camera.
     * This method updates the connection status, target observations, pose observations,
     * and tag IDs based on the current data from the Limelight.
     *
     * @param inputs The VisionIOInputs object to update with the latest vision data.
     */
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Set connection status to true
        inputs.connected = limelight.isConnected();

        // Update target observation with horizontal and vertical offsets
        inputs.latestTargetObservation = new TargetObservation(
            new Rotation2d(limelight.getHorizontalOffset()),
            new Rotation2d(limelight.getVerticalOffset())
        );

        // Retrieve the robot's pose estimate from the Limelight
        VisionObservation pose = limelight.getBotPoseEstimate();

        // Update pose observations
        inputs.poseObservations = new PoseObservation[] {
            new PoseObservation(
                pose.timestamp(),
                new Pose3d(pose.pose()),
                0.0, // Placeholder for error
                pose.tagCount(),
                pose.avgTagDist(),
                PoseObservationType.MEGATAG_1 // Set the type to MEGATAG_1 for Limelight
            )
        };

        // Set the tag IDs from the Limelight pose estimate
        // inputs.tagIds = limelight.getBotPoseEstimate().fiducialId();
    }

    @Override
    public void setPipeline(long pipelineID) {
        limelight.setPipeline(pipelineID);
    }
}

package frc.robot.sensors;

import frc.lib.team1648.Limelight;

public class RightLimelight {
    
    private Limelight limelight;

    public RightLimelight(String name) {
        limelight = new Limelight(name);
    }

    public VisionObservation getVisionObservation() {
        return limelight.getBotPoseEstimate();
    }
}

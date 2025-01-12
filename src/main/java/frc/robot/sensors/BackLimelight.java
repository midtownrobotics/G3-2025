package frc.robot.sensors;

import frc.lib.team1648.Limelight;

public class BackLimelight {
    
    private Limelight limelight;

    public BackLimelight(String name) {
        limelight = new Limelight(name);
    }

    public VisionObservation getVisionObservation() {
        return limelight.getBotPoseEstimate();
    }
}

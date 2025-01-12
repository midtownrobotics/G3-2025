package frc.robot.sensors;

import frc.lib.team1648.Limelight;

public class CoralLimelight {
  // enum: Pipeline

    Limelight limelight;

  public enum Pipeline {
    CORAL,
    APRILTAG;
  }

  private Pipeline currentPipeline;

  // getVisionObservation()
  public Integer getCoralOffset() {
    if (currentPipeline == Pipeline.APRILTAG || !limelight.isTargetSeen()) {
        return null;
    }

    return (int)limelight.getHorizontalOffset();

  }

  // setPipeLine(pipeline)
  public void setPipeline(Pipeline newPipeline) {
    currentPipeline = newPipeline;
  }
  // getPipeLine()
  public Pipeline getPipeline() {
    return currentPipeline;
  }
}

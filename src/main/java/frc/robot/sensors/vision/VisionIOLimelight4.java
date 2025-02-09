package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.Limelight;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class VisionIOLimelight4 extends VisionIOLimelight{
    Limelight limelight;

    @RequiredArgsConstructor
    public static enum IMUMode {
        MEGATAG_1(1.0),
        MEGATAG_2(2.0);

        @Getter private final double IMUModeID;
      }

    /**
     * Constructor for the Limelight4 VisionIO
     */
    public VisionIOLimelight4(String name, Supplier<Pose2d> poseSupplier) {
        super(name, poseSupplier);
        limelight  = super.getLimelight();
        limelight.setRobotYaw(null);
        limelight.setIMUMode(IMUMode.MEGATAG_1.IMUModeID);
    }
}

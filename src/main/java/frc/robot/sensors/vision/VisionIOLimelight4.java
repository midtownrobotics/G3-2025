package frc.robot.sensors.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Limelight;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class VisionIOLimelight4 extends VisionIOLimelight{
    private Limelight limelight;

    @RequiredArgsConstructor
    public static enum IMUMode {
        DISABLED_IMU_MODE(1.0),
        ENABLED_IMU_MODE(2.0);

        @Getter private final double IMUModeID;
    }

    private IMUMode currentMode = IMUMode.DISABLED_IMU_MODE;


    /**
     * Constructor for the Limelight4 VisionIO
     */
    public VisionIOLimelight4(String name, Supplier<Pose2d> poseSupplier) {
        super(name, poseSupplier);
        limelight  = super.getLimelight();
        limelight.setRobotYaw(poseSupplier.get().getRotation());
        limelight.setIMUMode(currentMode.getIMUModeID());
    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {
        super.updateInputs(inputs);
        if (DriverStation.isEnabled()) {
            limelight.setIMUMode(IMUMode.ENABLED_IMU_MODE.IMUModeID);
            currentMode = IMUMode.ENABLED_IMU_MODE;
        } else {
            limelight.setIMUMode(IMUMode.DISABLED_IMU_MODE.IMUModeID);
            currentMode = IMUMode.DISABLED_IMU_MODE;
        }
        
        
    }
}

package frc.lib.team1648;

import java.util.HashSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// getters and setters based on
// https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

public class Limelight {
    
    private static HashSet<String> takenNames;
    
    private NetworkTable table;

    public Limelight(String name) {
        if (takenNames.contains(name)) {
            // i forgot what kind of error logging should be here
        }
        takenNames.add(name);
        table = NetworkTableInstance.getDefault().getTable(name);
    }

    public static Pose2d toPose2d(double[] data) {
        if (data.length < 6) {
            return new Pose2d();
        }

        Translation2d tran2d = new Translation2d(data[0], data[1]);
        Rotation2d r2d = Rotation2d.fromDegrees(data[5]);

        return new Pose2d(tran2d, r2d);
    }

    public static double[] pose2dToArray(Pose2d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            0,
            0,
            0,
            pose.getRotation().getDegrees()
        };
    }

    public static double[] pose3dToArray(Pose3d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            Math.toDegrees(pose.getRotation().getX()),
            Math.toDegrees(pose.getRotation().getY()),
            Math.toDegrees(pose.getRotation().getZ())
        };
    }

    // private Pose3d toPose3d(double[] data) {
    //     if (data.length < 6) {
    //         return new Pose3d();
    //     }

    //     Translation2d tran2d = new Translation2d(data[0], data[1]);
    //     Rotation2d r2d = Rotation2d.fromDegrees(data[5]);

    //     return new Pose3d(
    //         new Translation3d(data[0], data[1], data[2]),
    //         new Rotation3d(
    //             Units.degreesToRadians(data[4]),
    //             Units.degreesToRadians(data[4]),
    //             Units.degreesToRadians(data[5])
    //         )
    //     );
    // }

    private void flush() {
        NetworkTableInstance.getDefault().flush();
    }

    private NetworkTableEntry get(String key) {
        return table.getEntry(key);
    }

    private long getLong(String key) {
        return get(key).getInteger(0);
    }

    private double getDouble(String key) {
        return get(key).getDouble(0);
    }

    private String getString(String key) {
        return get(key).getString("");
    }

    private long[] getLongArray(String key) {
        return get(key).getIntegerArray((long[])null);
    }

    private double[] getDoubleArray(String key) {
        return get(key).getDoubleArray((double[])null);
    }

    private String[] getStringArray(String key) {
        return get(key).getStringArray(null);
    }

    private long set(String key, long value) {
        get(key).setInteger(value);
        return value;
    }

    private double set(String key, double value) {
        get(key).setDouble(value);
        return value;
    }

    private String set(String key, String value) {
        get(key).setString(value);
        return value;
    }

    private long[] set(String key, long[] value) {
        get(key).setIntegerArray(value);
        return value;
    }

    private Long[] set(String key, Long[] value) {
        get(key).setIntegerArray(value);
        return value;
    }

    private double[] set(String key, double[] value) {
        get(key).setDoubleArray(value);
        return value;
    }

    private Double[] set(String key, Double[] value) {
        get(key).setDoubleArray(value);
        return value;
    }

    // begin the barrage of getters
    
    // use this method to index to get other stuff
    public double[] getStats() {
        return getDoubleArray("t2d");
    }

    public boolean isTargetSeen() {
        return getStats()[0] == 1;
    }

    public int getTargetCount() {
        return (int)getStats()[1];
    }

    public double getTargetLatency() {
        return getStats()[2];
    }

    public double getCaptureLatency() {
        return getStats()[3];
    }

    public double getHorizontalOffset() {
        return getStats()[4];
    }

    public double getVerticalOffset() {
        return getStats()[5];
    }

    public double getHorizontalOffsetFromPrincipalPixel() {
        return getStats()[6];
    }

    public double getVerticalOffsetFromPrincipalPixel() {
        return getStats()[7];
    }

    public double getTargetArea() {
        return getStats()[8];
    }

    public int getTargetId() {
        return (int)getStats()[9];
    }

    // public double getTargetClassIndexDetector() {
    //     return getStats()[10];
    // }

    // public double getTargetClassIndexClassifier() {
    //     return getStats()[11];
    // }

    // public double getTargetLongSidePixels() {
    //     return getStats()[12];
    // }

    // public double getTargetShortSidePixels() {
    //     return getStats()[13];
    // }

    // public double getTargetHorizontalExtentPixels() {
    //     return getStats()[14];
    // }

    // public double targetVerticalExtendPixels() {
    //     return getStats()[15];
    // }

    // public double targetSkewDegrees() {
    //     return getStats()[16];
    // }

    // public double getPipeLatency() {
    //     return getDouble("tl");
    // }

    public double getTotalLatency() {
        return getTargetLatency() + getCaptureLatency();
    }

    public byte getPipeIndex() {
        return (byte)getLong("getpipe");
    }

    public String getPipeType() {
        return getString("getpipetype");
    }

    // public String getJsonDump() {
    //     return getString("json");
    // }

    // public String getDetectorClass() {
    //     return getString("tclass");
    // }

    // public double[] getAvgColorUnderCrosshair() {
    //     return getDoubleArray("tc");
    // }

    // public double getHeartbeat() {
    //     return getDouble("hb");
    // }

    // public double[] getCrosshairs() {
    //     return getDoubleArray("crosshairs");
    // }

    // public String getPipeClassName() {
    //     return getString("tcclass");
    // }

    // public String getPipeDetectorName() {
    //     return getString("tdclass");
    // }

    // AprilTag stuff

    // public double[] getBotPose() {
    //     return getDoubleArray("botpose");
    // }

    public Pose2d getBotPoseBlue() {
        double[] res = getDoubleArray("botpose_wpiblue");
        return toPose2d(res);
    }

    // public double[] getBotPoseRed() {
    //     return getDoubleArray("botpose_wpired");
    // }

    // public double[] getBotPoseMegatag() {
    //     return getDoubleArray("botpose_orb");
    // }
    
    public Pose2d getBotPoseMegatagBlue() {
        double[] res = getDoubleArray("botpose_wpiblue");
        return toPose2d(res);
    }

    // public double[] getBotPoseMegatagRed() {
    //     return getDoubleArray("botpose_orb_wpired");
    // }

    // public double[] getCamPoseInTargetSpace() {
    //     return getDoubleArray("camerapose_targetspace");
    // }

    public Pose2d getTargetPoseInCamSpace() {
        return toPose2d(getDoubleArray("targetpose_cameraspace"));
    }

    public Pose2d getTargetPoseInRobotSpace() {
        return toPose2d(getDoubleArray("targetpose_robotspace"));
    }

    // public double[] getBotPoseInTargetSpace() {
    //     return getDoubleArray("botpose_targetspace");
    // }

    // public double[] getCamPoseInRobotSpace() {
    //     return getDoubleArray("camerapose_robotspace");
    // }

    public long getAprilTagId() {
        return getLong("tid");
    }

    public record Stddevs(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw
    ) {}

    public Stddevs getMegatagStddevs() {
        double[] res = getDoubleArray("stddevs");
        return new Stddevs(
            res[0],
            res[1],
            res[2],
            res[3],
            res[4],
            res[5]
        );
    }

    public Stddevs getMegatag2Stddevs() {
        double[] res = getDoubleArray("stddevs");
        return new Stddevs(
            res[6],
            res[7],
            res[8],
            res[9],
            res[10],
            res[11]
        );
    }

    // AprilTag setters

    public void setCamPoseInRobotSpace(Pose3d coords) {
        set("camerapose_robotspace_set", pose3dToArray(coords));
    }

    public void setTargetId(long id) {
        set("priorityid", id);
    }

    public void setRobotYaw(Rotation2d yaw) {
        set("robot_orientation_set", new double[] {yaw.getDegrees(), 0, 0, 0, 0, 0});
        flush();
    }

    public void setValidFiducialIds(double[] ids) {
        set("fiducial_id_filters_set", ids);
    }

    // public double[] setPOIOffset(double[] offsetXYZ) {
    //     set("fiducial_offset_set", offsetXYZ);
    // }

    // camera controls

    public enum LedState {
        CURRENTPIPELINE,
        OFF,
        BLINK,
        ON;
    }

    // public byte setLedState(long state) {
    //     return (byte)set("ledMode", state);
    // }

    public void setLedState(LedState state) {
        set("ledMode", state.ordinal());
    }

    public void setPipeline(long pipeline) {
        set("pipeline", pipeline);
    }

    // public enum StreamingMode {
    //     STANDARD,
    //     PiPMAIN,
    //     PiPSECONDARY;
    // }

    // public byte setStreamingMode(long mode) {
    //     return (byte)set("stream", mode);
    // }

    // public StreamingMode setStreamingMode(StreamingMode mode) {
    //     set("stream", mode.ordinal());
    //     return mode;
    // }

    public void setCrop(double[] x0x1y0y1) {
        set("crop", x0x1y0y1);
    }

}

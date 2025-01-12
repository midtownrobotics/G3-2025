package frc.lib.team1648;

import java.util.HashSet;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.sensors.VisionObservation;

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

    public VisionObservation getBotPoseEstimate() {
        DoubleArrayEntry poseEntry = table.getDoubleArrayTopic("botpose_wpiblue").getEntry(null);
        
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long timestamp = tsValue.timestamp;
        
        if (poseArray.length == 0) {
            // Handle the case where no data is available
            return null; // or some default PoseEstimate
        }
        
        var pose = toPose2d(poseArray);
        double latency = poseArray[6];
        int tagCount = (int)poseArray[7];
        // double tagSpan = poseArray[8];
        double tagDist = poseArray[9];
        // double tagArea = poseArray[10];
        
        // Convert server timestamp from microseconds to seconds and adjust for latency
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
    
        int[] rawFiducialIds = new int[tagCount];
        int valsPerFiducial = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
    
        if (poseArray.length != expectedTotalVals) {
            // Don't populate fiducials
        } else {
            for(int i = 0; i < tagCount; i++) {
                int baseIndex = 11 + (i * valsPerFiducial);
                int id = (int)poseArray[baseIndex];
                rawFiducialIds[i] = id;
            }
        }
        Stddevs stddevs = getMegatagStddevs();
        Matrix<N3, N1> stddevMatrix = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
        stddevMatrix.set(0, 0, stddevs.x);
        stddevMatrix.set(0, 1, stddevs.y);
        stddevMatrix.set(0, 2, Math.toRadians(stddevs.yaw));
        return new VisionObservation(pose, adjustedTimestamp, tagCount, tagDist, rawFiducialIds, stddevMatrix);
    }

    private void flush() {
        NetworkTableInstance.getDefault().flush();
    }

    private NetworkTableEntry get(String key) {
        return table.getEntry(key);
    }

    private long getLong(String key) {
        return get(key).getInteger(0);
    }

    @SuppressWarnings("unused")
    private double getDouble(String key) {
        return get(key).getDouble(0);
    }

    private String getString(String key) {
        return get(key).getString("");
    }

    @SuppressWarnings("unused")
    private long[] getLongArray(String key) {
        return get(key).getIntegerArray((long[])null);
    }

    private double[] getDoubleArray(String key) {
        return get(key).getDoubleArray((double[])null);
    }

    @SuppressWarnings("unused")
    private String[] getStringArray(String key) {
        return get(key).getStringArray(null);
    }

    private long set(String key, long value) {
        get(key).setInteger(value);
        return value;
    }

    @SuppressWarnings("unused")
    private double set(String key, double value) {
        get(key).setDouble(value);
        return value;
    }

    @SuppressWarnings("unused")
    private String set(String key, String value) {
        get(key).setString(value);
        return value;
    }

    @SuppressWarnings("unused")
    private long[] set(String key, long[] value) {
        get(key).setIntegerArray(value);
        return value;
    }

    @SuppressWarnings("unused")
    private Long[] set(String key, Long[] value) {
        get(key).setIntegerArray(value);
        return value;
    }

    private double[] set(String key, double[] value) {
        get(key).setDoubleArray(value);
        return value;
    }

    @SuppressWarnings("unused")
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

    public double getTotalLatency() {
        return getTargetLatency() + getCaptureLatency();
    }

    public byte getPipeIndex() {
        return (byte)getLong("getpipe");
    }

    public String getPipeType() {
        return getString("getpipetype");
    }

    // Apriltag stuff

    public Pose2d getBotPoseBlue() {
        double[] res = getDoubleArray("botpose_wpiblue");
        return toPose2d(res);
    }
    
    public Pose2d getBotPoseMegatagBlue() {
        double[] res = getDoubleArray("botpose_wpiblue");
        return toPose2d(res);
    }

    public Pose2d getTargetPoseInCamSpace() {
        return toPose2d(getDoubleArray("targetpose_cameraspace"));
    }

    public Pose2d getTargetPoseInRobotSpace() {
        return toPose2d(getDoubleArray("targetpose_robotspace"));
    }

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
        flush();
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

    // camera controls

    public enum LedState {
        CURRENTPIPELINE,
        OFF,
        BLINK,
        ON;
    }

    public void setLedState(LedState state) {
        set("ledMode", state.ordinal());
    }

    public void setPipeline(long pipeline) {
        set("pipeline", pipeline);
    }

    public void setCrop(double[] x0x1y0y1) {
        set("crop", x0x1y0y1);
    }

}

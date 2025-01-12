package frc.lib.team1648;

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
import java.util.HashSet;

/**
 * The Limelight class interacts with the Limelight Vision processing system to retrieve and process
 * data related to the robot's pose, vision targets, and camera settings.
 */
public class Limelight {

  // Keeps track of already used Limelight names to prevent duplication.
  private static HashSet<String> takenNames;

  // The NetworkTable used to interact with the Limelight system.
  private NetworkTable table;

  /**
   * Constructor that initializes the Limelight object with a given name.
   *
   * @param name The name of the Limelight instance.
   */
  public Limelight(String name) {
    if (takenNames.contains(name)) {
      // i forgot what kind of error logging should be here
    }
    takenNames.add(name);
    table = NetworkTableInstance.getDefault().getTable(name);
  }

  /**
   * Converts an array of doubles into a Pose2d object.
   *
   * @param data The array of doubles containing pose data.
   * @return A Pose2d object representing the pose.
   */
  public static Pose2d toPose2d(double[] data) {
    if (data.length < 6) {
      return new Pose2d();
    }

    Translation2d tran2d = new Translation2d(data[0], data[1]);
    Rotation2d r2d = Rotation2d.fromDegrees(data[5]);

    return new Pose2d(tran2d, r2d);
  }

  /**
   * Converts a Pose2d object into a corresponding array of doubles.
   *
   * @param pose The Pose2d object to convert.
   * @return A double array representing the pose.
   */
  public static double[] pose2dToArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), 0, 0, 0, pose.getRotation().getDegrees()};
  }

  /**
   * Converts a Pose3d object into a corresponding array of doubles.
   *
   * @param pose The Pose3d object to convert.
   * @return A double array representing the 3D pose.
   */
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

  /**
   * Retrieves the estimated bot pose from the Limelight system.
   *
   * @return A VisionObservation containing the pose and associated data, or null if no pose data is
   *     available.
   */
  public VisionObservation getBotPoseEstimate() {
    DoubleArrayEntry poseEntry = table.getDoubleArrayTopic("botpose_wpiblue").getEntry(null);

    TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    double[] poseArray = tsValue.value;
    long timestamp = tsValue.timestamp;

    if (poseArray.length == 0) {
      // Handle the case where no data is available
      return null;
    }

    var pose = toPose2d(poseArray);
    double latency = poseArray[6];
    int tagCount = (int) poseArray[7];
    double tagDist = poseArray[9];

    // Convert server timestamp from microseconds to seconds and adjust for latency
    double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

    int[] rawFiducialIds = new int[tagCount];
    int valsPerFiducial = 7;
    int expectedTotalVals = 11 + valsPerFiducial * tagCount;

    if (poseArray.length != expectedTotalVals) {
      // Don't populate fiducials
    } else {
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        int id = (int) poseArray[baseIndex];
        rawFiducialIds[i] = id;
      }
    }

    Stddevs stddevs = getMegatagStddevs();
    Matrix<N3, N1> stddevMatrix = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
    stddevMatrix.set(0, 0, stddevs.x);
    stddevMatrix.set(0, 1, stddevs.y);
    stddevMatrix.set(0, 2, Math.toRadians(stddevs.yaw));
    return new VisionObservation(
        pose, adjustedTimestamp, tagCount, tagDist, rawFiducialIds, stddevMatrix);
  }

  /** Flushes the network tables to send any updates to the Limelight system. */
  private void flush() {
    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Retrieves a network table entry for a given key.
   *
   * @param key The key for the network table entry.
   * @return The corresponding NetworkTableEntry.
   */
  private NetworkTableEntry get(String key) {
    return table.getEntry(key);
  }

  /**
   * Retrieves a long value from the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @return The long value from the network table.
   */
  private long getLong(String key) {
    return get(key).getInteger(0);
  }

  /**
   * Retrieves a double value from the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @return The double value from the network table.
   */
  @SuppressWarnings("unused")
  private double getDouble(String key) {
    return get(key).getDouble(0);
  }

  /**
   * Retrieves a string value from the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @return The string value from the network table.
   */
  private String getString(String key) {
    return get(key).getString("");
  }

  /**
   * Retrieves a long array value from the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @return The long array value from the network table.
   */
  @SuppressWarnings("unused")
  private long[] getLongArray(String key) {
    return get(key).getIntegerArray((long[]) null);
  }

  /**
   * Retrieves a double array value from the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @return The double array value from the network table.
   */
  private double[] getDoubleArray(String key) {
    return get(key).getDoubleArray((double[]) null);
  }

  /**
   * Retrieves a string array value from the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @return The string array value from the network table.
   */
  @SuppressWarnings("unused")
  private String[] getStringArray(String key) {
    return get(key).getStringArray(null);
  }

  /**
   * Sets a long value in the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @param value The value to set.
   * @return The value that was set.
   */
  private long set(String key, long value) {
    get(key).setInteger(value);
    return value;
  }

  /**
   * Sets a double value in the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @param value The value to set.
   * @return The value that was set.
   */
  @SuppressWarnings("unused")
  private double set(String key, double value) {
    get(key).setDouble(value);
    return value;
  }

  /**
   * Sets a string value in the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @param value The value to set.
   * @return The value that was set.
   */
  @SuppressWarnings("unused")
  private String set(String key, String value) {
    get(key).setString(value);
    return value;
  }

  /**
   * Sets a long array value in the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @param value The array of long values to set.
   * @return The array that was set.
   */
  @SuppressWarnings("unused")
  private long[] set(String key, long[] value) {
    get(key).setIntegerArray(value);
    return value;
  }

  /**
   * Sets a Long array value in the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @param value The array of Long values to set.
   * @return The array that was set.
   */
  @SuppressWarnings("unused")
  private Long[] set(String key, Long[] value) {
    get(key).setIntegerArray(value);
    return value;
  }

  /**
   * Sets a double array value in the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @param value The array of double values to set.
   * @return The array that was set.
   */
  private double[] set(String key, double[] value) {
    get(key).setDoubleArray(value);
    return value;
  }

  /**
   * Sets a Double array value in the network table for a given key.
   *
   * @param key The key for the network table entry.
   * @param value The array of Double values to set.
   * @return The array that was set.
   */
  @SuppressWarnings("unused")
  private Double[] set(String key, Double[] value) {
    get(key).setDoubleArray(value);
    return value;
  }

  // begin the barrage of getters

  /**
   * Retrieves various statistics from the Limelight system.
   *
   * @return A double array of statistics.
   */
  public double[] getStats() {
    return getDoubleArray("t2d");
  }

  /**
   * Checks if a target is currently seen by the Limelight camera.
   *
   * @return true if a target is seen, false otherwise.
   */
  public boolean isTargetSeen() {
    return getStats()[0] == 1;
  }

  /**
   * Retrieves the number of targets detected by the Limelight system.
   *
   * @return The number of targets detected.
   */
  public int getTargetCount() {
    return (int) getStats()[1];
  }

  /**
   * Retrieves the latency in the target detection process.
   *
   * @return The target latency in seconds.
   */
  public double getTargetLatency() {
    return getStats()[2];
  }

  /**
   * Retrieves the latency in capturing the target.
   *
   * @return The capture latency in seconds.
   */
  public double getCaptureLatency() {
    return getStats()[3];
  }

  /**
   * Retrieves the horizontal offset of the detected target from the camera center.
   *
   * @return The horizontal offset in pixels.
   */
  public double getHorizontalOffset() {
    return getStats()[4];
  }

  /**
   * Retrieves the vertical offset of the detected target from the camera center.
   *
   * @return The vertical offset in pixels.
   */
  public double getVerticalOffset() {
    return getStats()[5];
  }

  /**
   * Retrieves the horizontal offset from the principal pixel.
   *
   * @return The horizontal offset in pixels.
   */
  public double getHorizontalOffsetFromPrincipalPixel() {
    return getStats()[6];
  }

  /**
   * Retrieves the vertical offset from the principal pixel.
   *
   * @return The vertical offset in pixels.
   */
  public double getVerticalOffsetFromPrincipalPixel() {
    return getStats()[7];
  }

  /**
   * Retrieves the area of the detected target.
   *
   * @return The target area in pixels.
   */
  public double getTargetArea() {
    return getStats()[8];
  }

  /**
   * Retrieves the ID of the detected target.
   *
   * @return The target ID.
   */
  public int getTargetId() {
    return (int) getStats()[9];
  }

  /**
   * Retrieves the total latency (target + capture latency).
   *
   * @return The total latency in seconds.
   */
  public double getTotalLatency() {
    return getTargetLatency() + getCaptureLatency();
  }

  /**
   * Retrieves the current pipeline index.
   *
   * @return The pipeline index.
   */
  public byte getPipeIndex() {
    return (byte) getLong("getpipe");
  }

  /**
   * Retrieves the current pipeline type.
   *
   * @return The pipeline type as a string.
   */
  public String getPipeType() {
    return getString("getpipetype");
  }

  // Apriltag related methods...

  /**
   * Retrieves the bot pose for the blue team from the Limelight system.
   *
   * @return A Pose2d object representing the bot pose.
   */
  public Pose2d getBotPoseBlue() {
    double[] res = getDoubleArray("botpose_wpiblue");
    return toPose2d(res);
  }

  /**
   * Retrieves the bot pose for the Megatag blue team from the Limelight system.
   *
   * @return A Pose2d object representing the bot pose.
   */
  public Pose2d getBotPoseMegatagBlue() {
    double[] res = getDoubleArray("botpose_wpiblue");
    return toPose2d(res);
  }

  /**
   * Retrieves the target pose in camera space.
   *
   * @return A Pose2d object representing the target pose in camera space.
   */
  public Pose2d getTargetPoseInCamSpace() {
    return toPose2d(getDoubleArray("targetpose_cameraspace"));
  }

  /**
   * Retrieves the target pose in robot space.
   *
   * @return A Pose2d object representing the target pose in robot space.
   */
  public Pose2d getTargetPoseInRobotSpace() {
    return toPose2d(getDoubleArray("targetpose_robotspace"));
  }

  /**
   * Retrieves the AprilTag ID.
   *
   * @return The AprilTag ID.
   */
  public long getAprilTagId() {
    return getLong("tid");
  }

  /** A record class that holds standard deviation values for x, y, z, roll, pitch, and yaw. */
  public record Stddevs(double x, double y, double z, double roll, double pitch, double yaw) {}

  /**
   * Retrieves the standard deviations for Megatags from the Limelight system.
   *
   * @return An Stddevs object containing standard deviations for the Megatag data.
   */
  public Stddevs getMegatagStddevs() {
    double[] res = getDoubleArray("stddevs");
    return new Stddevs(res[0], res[1], res[2], res[3], res[4], res[5]);
  }

  /**
   * Retrieves the standard deviations for the second Megatag from the Limelight system.
   *
   * @return An Stddevs object containing standard deviations for the second Megatag.
   */
  public Stddevs getMegatag2Stddevs() {
    double[] res = getDoubleArray("stddevs");
    return new Stddevs(res[6], res[7], res[8], res[9], res[10], res[11]);
  }

  /**
   * Sets the camera pose in robot space.
   *
   * @param coords The Pose3d representing the camera pose in robot space.
   */
  public void setCamPoseInRobotSpace(Pose3d coords) {
    set("camerapose_robotspace_set", pose3dToArray(coords));
    flush();
  }

  /**
   * Sets the target ID in the Limelight system.
   *
   * @param id The target ID to set.
   */
  public void setTargetId(long id) {
    set("priorityid", id);
  }

  /**
   * Sets the robot's yaw orientation.
   *
   * @param yaw The Rotation2d representing the robot's yaw.
   */
  public void setRobotYaw(Rotation2d yaw) {
    set("robot_orientation_set", new double[] {yaw.getDegrees(), 0, 0, 0, 0, 0});
    flush();
  }

  /**
   * Sets the valid fiducial IDs in the Limelight system.
   *
   * @param ids The array of valid fiducial IDs.
   */
  public void setValidFiducialIds(double[] ids) {
    set("fiducial_id_filters_set", ids);
  }

  /** Enum representing the different LED states for the Limelight camera. */
  public enum LedState {
    CURRENTPIPELINE,
    OFF,
    BLINK,
    ON;
  }

  /**
   * Sets the LED state for the Limelight camera.
   *
   * @param state The LED state to set.
   */
  public void setLedState(LedState state) {
    set("ledMode", state.ordinal());
  }

  /**
   * Sets the pipeline index for the Limelight camera.
   *
   * @param pipeline The pipeline index to set.
   */
  public void setPipeline(long pipeline) {
    set("pipeline", pipeline);
  }

  /**
   * Sets the crop area for the Limelight camera.
   *
   * @param x0x1y0y1 The array representing the crop area in the format [x0, x1, y0, y1].
   */
  public void setCrop(double[] x0x1y0y1) {
    set("crop", x0x1y0y1);
  }
}

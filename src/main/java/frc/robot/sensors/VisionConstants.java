// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // Camera names, must match names configured on coprocessor
  public static final String kModuleTagCameraName = "limelight-nick";
  public static final String kElevatorTagCameraName = "limelight-woody";
  public static final String kPoleTagCameraName = "limelight-gray";
  public static final String kIntakeClassifierCameraName = "limelight-ankit";

      public static final Transform3d kModuleTagRobotToCamera = new Transform3d(
        new Translation3d(Inches.of(8.431), Inches.of(12.458), Inches.of(8.053)),
        new Rotation3d(Degrees.zero(), Degrees.of(-15), Degrees.of(15))
    );

      public static final Transform3d kElevatorTagRobotToCamera = new Transform3d(
        new Translation3d(Inches.of(-2.998), Inches.of(-12.74), Inches.of(10.918)),
        new Rotation3d(Degrees.zero(), Degrees.of(-10), Degrees.of(-20))
    );

    public static final Transform3d kPoleTagRobotToCamera = new Transform3d(
        new Translation3d(Inches.of(9.549), Inches.of(4.813), Inches.of(13.162)),
        new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero())
    );

    public static final Transform3d kIntakeClassifierRobotToCamera = new Transform3d(
        new Translation3d(Inches.of(-8.451), Inches.of(-6.377), Inches.of(37.864)),
        new Rotation3d(Degrees.zero(), Degrees.of(-37), Degrees.of(87.5))
    );

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.3;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.09; // Meters
  public static double angularStdDevBaseline = 0.3; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}

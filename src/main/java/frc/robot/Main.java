// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    // Pose2d robotPoseAtBranchE = new Pose2d(4.963, 2.702, Rotation2d.fromDegrees(120));

    // Pose2d branchEPose = FieldConstants.Reef.branchPositions2d.get(4).get(ReefLevel.L1);

    // Transform2d offset = robotPoseAtBranchE.minus(branchEPose);

    // for (int i = 0; i < 12; i++) {
    //   Pose2d branchPose = FieldConstants.Reef.branchPositions2d.get(i).get(ReefLevel.L1);

    //   Pose2d robotPose = branchPose.transformBy(offset);

    //   System.out.println("Pose for branch " + i + " is " + String.format("Translation2d(X: %.3f, Y: %.3f, Theta: %.3f)", robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees()));

    // }
    RobotBase.startRobot(Robot::new);
  }
}

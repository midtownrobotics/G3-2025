package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team6328.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotViz extends VirtualSubsystem {

    private final Supplier<Pose2d> robotPose;
    private final Supplier<Angle> intakePosition;
    private final Supplier<Distance> elevatorPosition;
    private final Supplier<Angle> clawPosition;

    public RobotViz(Supplier<Pose2d> robotPose, Supplier<Angle> intakePosition, Supplier<Distance> elevatorPosition, Supplier<Angle> clawPosition) {
        this.robotPose = robotPose;
        this.intakePosition = intakePosition;
        this.elevatorPosition = elevatorPosition;
        this.clawPosition = clawPosition;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("RobotViz/robotPose", robotPose.get());

        Pose3d intakePose = new Pose3d(
            new Translation3d(0, 0.23, 0.228),
            new Rotation3d(Degrees.zero(), intakePosition.get().unaryMinus(), Degrees.of(90))
        );

        Distance elevatorDistance = elevatorPosition.get();

        Pose3d stage1Pose = new Pose3d(
            new Translation3d(Meters.zero(), Meters.zero(), elevatorDistance.div(3)),
            new Rotation3d()
        );

        Pose3d stage2Pose = new Pose3d(
            new Translation3d(Meters.zero(), Meters.zero(), elevatorDistance.div(2)),
            new Rotation3d()
        );

        Pose3d carriagePose = new Pose3d(
            new Translation3d(Meters.zero(), Meters.zero(), elevatorDistance),
            new Rotation3d()
        );

        Pose3d clawPose = new Pose3d(
            new Translation3d(0.175, 0, 0.375),
            new Rotation3d(Degrees.zero(), clawPosition.get().unaryMinus(), Degrees.zero())
        );

        Pose3d[] componentPoses = new Pose3d[]{
            intakePose, stage1Pose, stage2Pose, carriagePose, clawPose
        };

        Logger.recordOutput("RobotViz/ComponentPoses", componentPoses);
    }

}

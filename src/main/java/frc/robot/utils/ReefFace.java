package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.AllianceFlipUtil;
import frc.robot.subsystems.drivetrain.Drive;

public enum ReefFace {
    /** The face closest to the driver */
    AB,

    /** The face on the right, closer to the driver */
    CD,

    /** The face on the right, further from the driver */
    EF,

    /** The face furthest from the driver */
    GH,

    /** The face on the left, further from the driver */
    IJ,

    /** The face on the left, closer to the driver */
    KL;

    /** Create a ReefFace from a POV value */
    public static ReefFace fromPOV(int pov) {
        switch (pov) {
            case 180:
                return AB;
            case 135:
            case 90:
                return CD;
            case 45:
                return EF;
            case 0:
                return GH;
            case 315:
                return IJ;
            case 270:
            case 225:
                return KL;
            default:
                return null;
        }
    }

    /** Is the algae on this face high or low? */
    public boolean isAlgaePositionHigh() {
        switch (this) {
            case AB:
            case EF:
            case IJ:
                return true;
            default:
                return false;
        }
    }

    public int toIndex() {
        switch (this) {
            case AB: return 0;
            case CD: return 1;
            case EF: return 2;
            case GH: return 3;
            case IJ: return 4;
            case KL: return 5;
            default: return 5;
        }
    }

    public static ReefFace getClosestReefFace(Drive drive) {
        ReefFace closestFace = null;
        Distance closestDistance = Meters.of(Double.MAX_VALUE);
        Pose2d currentPose = drive.getPose();

        for (ReefFace face : ReefFace.values()) {
            Pose2d rawReefFacePose = FieldConstants.Reef.centerFaces[face.ordinal()];
            Pose2d reefFacePose = AllianceFlipUtil.apply(rawReefFacePose);
            Distance distance = Meters
                    .of(reefFacePose.getTranslation().getDistance(currentPose.getTranslation()));
            if (distance.lt(closestDistance)) {
                closestFace = face;
                closestDistance = distance;
            }
        }

        return closestFace;
    }

    public static Distance getClosestReefFaceDistance(Drive drive) {
        Distance closestDistance = Meters.of(Double.MAX_VALUE);
        Pose2d currentPose = drive.getPose();

        for (ReefFace face : ReefFace.values()) {
            Pose2d rawReefFacePose = FieldConstants.Reef.centerFaces[face.ordinal()];
            Pose2d reefFacePose = AllianceFlipUtil.apply(rawReefFacePose);
            Distance distance = Meters
                    .of(reefFacePose.getTranslation().getDistance(currentPose.getTranslation()));
            if (distance.lt(closestDistance)) {
                closestDistance = distance;
            }
        }

        return closestDistance;
    }

}

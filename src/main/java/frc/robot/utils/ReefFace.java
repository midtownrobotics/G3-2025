package frc.robot.utils;

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
}

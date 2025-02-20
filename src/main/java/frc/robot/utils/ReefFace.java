package frc.robot.utils;

public enum ReefFace {
    AB,
    CD,
    EF,
    GH,
    IJ,
    KL;

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

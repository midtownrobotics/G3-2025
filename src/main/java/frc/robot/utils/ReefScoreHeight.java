package frc.robot.utils;

public enum ReefScoreHeight {
    L4,
    L3,
    L2,
    L1;

    public int toIndex() {
        switch (this) {
            case L1: return 0;
            case L2: return 1;
            case L3: return 2;
            case L4: return 3;
            default: return 3;
        }
    }
}

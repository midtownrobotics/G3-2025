package frc.robot.utils;

public enum ReefFaceSide {
    LEFT,
    RIGHT;

    public int toIndex() {
        switch (this) {
            case LEFT: return 0;
            case RIGHT: return 1;
            default: return 1;
        }
    }
}

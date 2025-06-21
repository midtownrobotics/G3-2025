package frc.robot.utils;

import frc.robot.controls.CoralMode;

public enum ReefScoreHeight {
    L4,
    L3,
    L2,
    L1;

    public int toIndex() {
        switch (this) {
            case L2: return 0;
            case L3: return 1;
            case L4: return 2;
            default: return 2;
        }
    }

    public CoralMode toCoralMode() {
        switch (this) {
            case L1: return CoralMode.L1;
            case L2: return CoralMode.L2;
            case L3: return CoralMode.L3;
            case L4: return CoralMode.L4;
            default: return CoralMode.L4;
        }
    }

    public static ReefScoreHeight fromCoralMode(CoralMode coralMode) {
        switch (coralMode) {
            case L1: return ReefScoreHeight.L1;
            case L2: return ReefScoreHeight.L2;
            case L3: return ReefScoreHeight.L3;
            case L4: return ReefScoreHeight.L4;
            default: return ReefScoreHeight.L1;
        }
    }
}

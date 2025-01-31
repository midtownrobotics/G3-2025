package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode MODE;
    private static final boolean enableReplay = false;

    static {
        if (RobotBase.isReal()) MODE = Mode.REAL;
        else if (enableReplay) MODE = Mode.REPLAY;
        else MODE = Mode.SIM;
    }
}

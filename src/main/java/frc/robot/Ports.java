package frc.robot;

// Define port variables
public class Ports {
    public static class AlgaeClaw {
        public static final int WristMotor = 0;
        public static final int WristEncoder = 1;
        public static final int AlgaeClawRoller = 2;
    }

    public static class Elevator {
        public static final int LeftWinchMotor = 3;
        public static final int RightWinchMotor = 4;
        public static final int LeftWinchEncoder = -1;
        public static final int RightWinchEncoder = -1;
    }

    public static class CoralIntake {
        public static final int Belt = 5;
        public static final int PivotMotor = 6;
        public static final int PivotEncoder = 7;
        public static final int CoralIntakeRoller = 8;
    }

    public static class CoralOuttake {
        public static final int Roller = 9;
    }
}

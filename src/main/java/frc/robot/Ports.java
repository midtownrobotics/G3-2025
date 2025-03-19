package frc.robot;

// Define port variables
public class Ports {
    public static class Elevator {
        public static final int LeftWinchMotor = 62;
        public static final int RightWinchMotor = 61;
        public static final int WinchEncoder = 1;
        public static final int LockServo = 4;
    }

    public static class CoralIntake {
        public static final int belt = 50;
        public static final int pivotMotor = 52;
        public static final int pivotEncoder = 9;
        public static final int coralIntakeRoller = 51;
        public static final int centerSensor = 4;
        public static final int handoffSensor = 3;
    }

    public static class CoralOuttake {
        public static final int roller = 31;
        public static final int pivotMotor = 57;
        public static final int pivotEncoder = 2;
    }
}

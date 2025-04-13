package frc.robot;

// Define port variables
public class Ports {
    public static final int LED = 5;

    public static class Elevator {
        public static final int LeftWinchMotor = 62;
        public static final int RightWinchMotor = 61;
        public static final int WinchEncoder = 1;
        public static final int LockServo = 4;
        public static final int zeroSensor = 99; // TODO
    }

    public static class CoralIntake {
        public static final int belt = 50;
        public static final int pivotMotor = 52;
        public static final int pivotEncoder = 5;
        public static final int coralIntakeRoller = 51;
        public static final int centerSensor = 4;
        public static final int handoffSensor = 2;
        public static final int upperZeroSensor = 6;
        public static final int lowerZeroSensor = 98; // TODO
    }

    public static class CoralOuttake {
        public static final int roller = 58;
        public static final int pivotMotor = 57;
        public static final int pivotEncoder = 0;
    }
}

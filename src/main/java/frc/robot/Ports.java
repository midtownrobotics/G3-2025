package frc.robot;

// Define port variables
public class Ports {
    public static class AlgaeClaw {
        public static final int wristMotor = 0;
        public static final int wristEncoder = 1;
        public static final int algaeClawRoller = 2;
    }

    public static class Elevator {
        public static final int LeftWinchMotor = 3;
        public static final int RightWinchMotor = 4;
        public static final int LeftWinchEncoder = -1;
        public static final int RightWinchEncoder = -1;
    }

    public static class CoralIntake {
        public static final int belt = 5;
        public static final int pivotMotor = 6;
        public static final int pivotEncoder = 7;
        public static final int coralIntakeRoller = 8;
        public static final int centerSensor = 0;
        public static final int handoffSensor = 1;
    }

    public static class CoralOuttake {
        public static final int roller = 9;
    }
}

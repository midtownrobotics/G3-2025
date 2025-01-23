package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controls {

  public static double DRIVER_JOYSTICK_THRESHHOLD = 0.1;

  /** Determines if the driver joystick is in the controller deadzone. */
  default boolean isDriverControlInDeadzone(double driveX, double driveY, double driveOmega) {
    return Math.sqrt(Math.pow(driveX, 2) + Math.pow(driveY, 2) + Math.pow(driveOmega, 2))
        > DRIVER_JOYSTICK_THRESHHOLD;
  }

  // Driver Controls

  /** Gets the drive forward/backward axis. */
  public double getDriveForward();
  /** Gets the drive left/right axis. */
  public double getDriveLeft();
  /** Gets the rotation axis. */
  public double getDriveRotation();

  /** Resets the drive heading. */
  public Trigger resetDriveHeading();
  /** Brakes the drivetrain. "X" Mode. */
  public Trigger driveBrake();
  /** Locks on to game piece. */
  public Trigger gamePieceLock();
  /** Locks to left position on the reef. */
  public Trigger leftPositionLock();
  /** Locks to right position on the reef. */
  public Trigger rightPositionLock();
  /** Locks to true right position on reef for algae intake. */
  public Trigger reefAlgaePositionLock();

  // Operator Controls

  /** Intake a coral from the ground. */
  public Trigger groundIntakeCoral();
  /** Vomit a coral from the ground. */
  public Trigger groundVomitCoral();
  /** Intake a coral from the source. */
  public Trigger sourceIntakeCoral();
  /** Vomit a coral from the source. */
  public Trigger sourceVomitCoral();

  /** Intake algae from the ground. */
  public Trigger groundIntakeAlgae();
  /** Vomits algae from the ground. */
  public Trigger groundVomitAlgae();
  /** Intakes stacked algae. */
  public Trigger stackedIntakeAlgae();
  /** Vomits stacked algae. */
  public Trigger stackedVomitAlgae();

  /** Prepares to score algae. */
  public Trigger prepareScoreAlgae();
  /** Prepares to score coral. */
  public Trigger prepareScoreCoral();
  /** Hands off coral. */
  public Trigger handoffCoral();
  /** Scores a game piece. */
  public Trigger scoreGamePiece();

  /** Gets the coral mode. */
  public CoralMode getCoralMode();
  /** Gets the algae mode. */
  public AlgaeMode getAlgaeMode();

  /** Activates the climbing mechanism. */
  public Trigger climb();

  /** !!!!!!!!!!!!!!danger!!!!!!!!!!!!!! */
  public Trigger panic();

  /** Switches the robot to manual mode. */
  public Trigger setManualMode();

  // Manual Controls

  public double getElevatorAxis();
  public Trigger wristForward();
  public Trigger wristBackward();
  public Trigger outtakeShoot();
  public Trigger algaeClawIntake();
  public Trigger coralForward();
  public Trigger coralBackward();
  public Trigger coralIntakeRun();
  public Trigger coralIntakeReverse();
}

package frc.robot.controls;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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


  /** drive sysid quasistatic forwards */
  public Trigger driveQuasistaticForwards();
  /** drive sysid quasistatic backwards */
  public Trigger driveQuasistaticBackwards();
  /** drive sysid dynamic forwards */
  public Trigger driveDynamicForwards();
  /** drive sysid dynamic backwards */
  public Trigger driveDynamicBackwards();

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
  /** Reverses a game piece. */
  public Trigger reverseGamePiece();

  /** Moves elevator into position to score L1 */
  public Trigger prepareScoreCoralL1();
  /** Moves elevator into position to score L2 */
  public Trigger prepareScoreCoralL2();
  /** Moves elevator into position to score L3 */
  public Trigger prepareScoreCoralL3();
  /** Moves elevator into position to score L4 */
  public Trigger prepareScoreCoralL4();
  /** Scores or removes algae based on coral mode*/
  public Trigger algae();

  /** Sets the algae claw to barge mode */
  public Trigger algaeModeBarge();

  /** Sets the algae claw to processor mode */
  public Trigger algaeModeProcessor();

  /** Activates the climbing mechanism. */
  public Trigger climb();

  /** !!!!!!!!!!!!!!danger!!!!!!!!!!!!!! */
  public Trigger panic();

  /** Switches the robot to manual mode. */
  public Trigger setManualMode();

  // Manual Controls

  /**
   * Manual Mode - Get Elevator Movement Axis
   */
  public double getElevatorAxis();

  /**
   * Manual Mode - Get Wrist Movement Axis
   */
  public double getWristAxis();

  /**
   * Manual Mode - Outtake Coral
   */
  public Trigger outtakeShoot();

  /**
   * Manual Mode - Intake Algae Claw
   */
  public Trigger algaeClawIntake();

  /**
   * Manual Mode - Reverse Algae Claw
   */
  public Trigger algaeClawReverse();

  /**
   * Manual Mode - Move Intake Forward
   */
  public Trigger coralForward();

  /**
   * Manual Mode - Move Intake Backward
   */
  public Trigger coralBackward();

  /**
   * Manual Mode - Run Coral Intake Rollers
   */
  public Trigger coralIntakeRun();

  /**
   * Manual Mode - Reverses Coral Intake Rollers
   */
  public Trigger coralIntakeReverse();

  /** Button to align to the left-most branch of a reef face. */
  public Trigger alignToReefLeftBranch();

  /** Button to align to the right-most branch of a reef face. */
  public Trigger alignToReefRightBranch();

  /** Button to align to the branch on the reef */
  public Trigger alignToBranchReef();

  /** Button to align to the center on the reef */
  public Trigger alignToAlgaeReef();

  /** Gets the POV value of the driver controller */
  public int getDriverPOV();

  /** Sets the rumble of the driver controller */
  public void setDriverRumble(double rumble);

  /** Sets the rumble of the operator controller */
  public void setOperatorRumble(double rumble);

  /** */
  public Trigger increaseElevatorOffset();

  /** */
  public Trigger decreaseElevatorOffset();

  /** zero elevator */
  public Trigger zeroElevator();

  /** Returns a command that sets the rumble of both controllers */
  public default Command setRumbleCommand(double rumble) {
    return Commands.runOnce(() -> {
      setDriverRumble(rumble);
      setOperatorRumble(rumble);
    });
  }

  /** Returns a command that sets the rumble of both controllers for a duration */
  public default Command setRumbleCommand(double rumble, Time duration) {
    return Commands.sequence(
      setRumbleCommand(1.0),
      Commands.waitSeconds(0.5),
      setRumbleCommand(0.0)
    );
  }
}

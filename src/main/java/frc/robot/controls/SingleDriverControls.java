package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class SingleDriverControls {

  private final IOProtectionXboxController controller;

  public static double DRIVER_JOYSTICK_THRESHHOLD = 0.1;
  public static double DRIVER_TRIGGER_PRESSED_THRESHHOLD = 0.5;

  @Getter
  private boolean groundIntakeMode = true;

  boolean isDriverControlInDeadzone(double driveX, double driveY, double driveOmega) {
    return Math.sqrt(Math.pow(driveX, 2) + Math.pow(driveY, 2) + Math.pow(driveOmega, 2))
        > DRIVER_JOYSTICK_THRESHHOLD;
  }

  /** Initalizes Xbox controls for SDC. */
  public SingleDriverControls(int port) {
    controller = new IOProtectionXboxController(port);

    controller.start().onTrue(Commands.run(() -> groundIntakeMode = !groundIntakeMode));
  }

  @AutoLogOutput
  /** Determines if the driver joystick is in the controller deadzone. */
  public boolean isDriverControlInDeadzone() {
    return isDriverControlInDeadzone(
      controller.getLeftX(), controller.getLeftY(), controller.getRightX());
  }

  /** Gets the drive forward axis. */
  public double getDriveForward() {
    double deadzoneApplied = MathUtil.applyDeadband(controller.getLeftY(), DRIVER_JOYSTICK_THRESHHOLD);
    return (isDriverControlInDeadzone()
    ? -Math.signum(deadzoneApplied)
    * Math.abs(Math.pow(deadzoneApplied, 1))
    : 0);
  }

  /** Gets the drive left/right axis. */
  public double getDriveLeft() {
    double deadzoneApplied = MathUtil.applyDeadband(controller.getLeftX(), DRIVER_JOYSTICK_THRESHHOLD);
    return (isDriverControlInDeadzone()
    ? -Math.signum(deadzoneApplied)
    * Math.abs(Math.pow(deadzoneApplied, 1))
    : 0);
  }

  /** Gets the drive rotation axis. */
  public double getDriveRotation() {
    double deadzoneApplied = MathUtil.applyDeadband(controller.getRightX(), DRIVER_JOYSTICK_THRESHHOLD);
    return isDriverControlInDeadzone()
    ? -Math.signum(deadzoneApplied)
    * Math.abs(Math.pow(deadzoneApplied, 2))
    : 0;
  }

  /** Sets coralMode to L1. */
  public Trigger prepareScoreCoralL1() {
    return controller.b();
  }

  /** Sets coralMode to L2. */
  public Trigger prepareScoreCoralL2() {
    return controller.a();
  }

  /** Sets coralMode to L3. */
  public Trigger prepareScoreCoralL3() {
    return controller.x();
  }

  /** Sets coralMode to L4. */
  public Trigger prepareScoreCoralL4() {
    return controller.y();
  }

  /** Aligns to processor or algae. */
  public Trigger algaeAutoAlign() {
    return controller.leftBumper().and(controller.rightBumper());
  }

  /** Aligns to left or right branch and shoots coral. */
  public Trigger coralAutoAlign() {
    return controller.leftBumper().and(controller.rightBumper().negate()).or(controller.rightBumper().and(controller.leftBumper().negate()));
  }

  /** Whether the left branch is selected. Otherwise, right is assumed selected. */
  public BooleanSupplier leftBranchSelectedSupplier() {
    return controller.leftBumper().and(controller.rightBumper().negate());
  }

  /** Manually shoots piece. Required for algae. */
  public Trigger manualShoot() {
    return controller.rightTrigger().and(controller.leftTrigger().negate());
  }

  /** Intakes coral from the source or the ground and puts coralMode to L1. */
  public Trigger intakeL1() {
    return controller.leftTrigger().and(controller.rightTrigger().negate());
  }

  /** Intakes coral from the source or the ground and puts coralMode. */
  public Trigger intake() {
    return controller.rightTrigger().and(controller.leftTrigger().negate());
  }

  /** Climbs!! */
  public Trigger climb() {
    return controller.rightTrigger().and(controller.leftTrigger());
  }

  /** Increases elevator offset. */
  public Trigger increaseElevatorOffset() {
    return controller.povUp();
  }

  /** Decreases elevator offset. */
  public Trigger decreaseElevatorOffset() {
    return controller.povDown();
  }
}

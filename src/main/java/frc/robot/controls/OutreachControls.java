package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.IOProtectionXboxController;
import org.littletonrobotics.junction.AutoLogOutput;

public class OutreachControls {
    private final IOProtectionXboxController overrideController;
    private final IOProtectionXboxController restrictedController;

    @AutoLogOutput
    private boolean allowRestrictedControls = false;

    public static double DRIVER_JOYSTICK_THRESHHOLD = 0.1;
    public static double DRIVER_TRIGGER_PRESSED_THRESHHOLD = 0.5;

    @AutoLogOutput
    private boolean groundIntakeMode = true;

    boolean isDriverControlInDeadzone(double driveX, double driveY, double driveOmega) {
        return Math.sqrt(Math.pow(driveX, 2) + Math.pow(driveY, 2) + Math.pow(driveOmega, 2)) > DRIVER_JOYSTICK_THRESHHOLD;
    }

    public OutreachControls(int overrideControllerPort, int restrictedControllerPort) {
        overrideController = new IOProtectionXboxController(overrideControllerPort);
        restrictedController = new IOProtectionXboxController(restrictedControllerPort);

        overrideController.rightTrigger().onTrue(Commands.runOnce(() -> allowRestrictedControls = true))
                .onFalse(Commands.runOnce(() -> allowRestrictedControls = false));
    }

    @AutoLogOutput
    public boolean isDriverControlInDeadzone() {
        if (allowRestrictedControls) {
            return isDriverControlInDeadzone(restrictedController.getLeftX(), restrictedController.getLeftY(),
                    restrictedController.getRightX());
        }
        return isDriverControlInDeadzone(overrideController.getLeftX(), overrideController.getLeftY(),
                overrideController.getRightX());
    }

    @AutoLogOutput
    public double getDriveForward() {
        double deadzoneApplied = MathUtil.applyDeadband(overrideController.getLeftY(), DRIVER_JOYSTICK_THRESHHOLD);
        if (allowRestrictedControls) {
            deadzoneApplied = MathUtil.applyDeadband(restrictedController.getLeftY(), DRIVER_JOYSTICK_THRESHHOLD);
        }

        return (isDriverControlInDeadzone()
                ? -Math.signum(deadzoneApplied)
                        * Math.abs(Math.pow(deadzoneApplied, 1))
                : 0);
    }

    @AutoLogOutput
    public double getDriveLeft() {
        double deadzoneApplied = MathUtil.applyDeadband(overrideController.getLeftX(), DRIVER_JOYSTICK_THRESHHOLD);
        if (allowRestrictedControls) {
            deadzoneApplied = MathUtil.applyDeadband(restrictedController.getLeftX(), DRIVER_JOYSTICK_THRESHHOLD);
        }
        return (isDriverControlInDeadzone()
                ? -Math.signum(deadzoneApplied)
                        * Math.abs(Math.pow(deadzoneApplied, 1))
                : 0);
    }

    @AutoLogOutput
    public double getDriveRotation() {
        double deadzoneApplied = MathUtil.applyDeadband(overrideController.getRightX(), DRIVER_JOYSTICK_THRESHHOLD);
        if (allowRestrictedControls) {
            deadzoneApplied = MathUtil.applyDeadband(restrictedController.getRightX(), DRIVER_JOYSTICK_THRESHHOLD);
        }

        return (isDriverControlInDeadzone()
                ? -Math.signum(deadzoneApplied)
                        * Math.abs(Math.pow(deadzoneApplied, 1))
                : 0);
    }

    public void setRumble(double rumbliness) {
        restrictedController.setRumble(RumbleType.kBothRumble, rumbliness);
        overrideController.setRumble(RumbleType.kBothRumble, rumbliness);
    }

    @AutoLogOutput
    public Trigger raiseElevator() {
        return restrictedController.a().and(() -> allowRestrictedControls)
               .or(overrideController.a().and(() -> !allowRestrictedControls));
    }

    @AutoLogOutput
    public Trigger shoot() {
        return restrictedController.leftTrigger().and(() -> allowRestrictedControls)
               .or(overrideController.leftTrigger().and(() -> !allowRestrictedControls));
    }

    @AutoLogOutput
    public Trigger intake() {
        return restrictedController.leftBumper().and(() -> allowRestrictedControls)
               .or(overrideController.leftBumper().and(() -> !allowRestrictedControls));
    }

}

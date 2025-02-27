package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.DoublePressTracker;
import frc.lib.IOProtectionXboxController;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class MatchXboxControls implements Controls {

  private final IOProtectionXboxController driverController;
  private final IOProtectionXboxController operatorController;

  private LoggedNetworkBoolean manualMode = new LoggedNetworkBoolean("Controls/manualMode", false);

  private BooleanSupplier getManualMode = () -> manualMode.get();
  private BooleanSupplier getNotManualMode = () -> !manualMode.get();

  /** Initalizes Xbox controls for matches. */
  public MatchXboxControls(int driverPort, int operatorPort) {
    driverController = new IOProtectionXboxController(driverPort);
    operatorController = new IOProtectionXboxController(operatorPort);
  }

  /** Determines if the driver joystick is in the controller deadzone. */
  private boolean isDriverControlInDeadzone() {
    return Controls.super.isDriverControlInDeadzone(
        driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX());
  }

  // Driver controls

  @Override
  public double getDriveForward() {
    return -MathUtil.applyDeadband(driverController.getLeftY(), DRIVER_JOYSTICK_THRESHHOLD);
    // return (isDriverControlInDeadzone()
    //     ? -Math.signum(driverController.getLeftY())
    //         * Math.abs(Math.pow(driverController.getLeftY(), 2))
    //     : 0);
  }

  @Override
  public double getDriveLeft() {
    return -MathUtil.applyDeadband(driverController.getLeftX(), DRIVER_JOYSTICK_THRESHHOLD);
    // return (isDriverControlInDeadzone()
    //     ? -Math.signum(driverController.getLeftX())
    //         * Math.abs(Math.pow(driverController.getLeftX(), 2))
    //     : 0);
  }

  @Override
  public double getDriveRotation() {
    return -MathUtil.applyDeadband(driverController.getRightX(), DRIVER_JOYSTICK_THRESHHOLD);
    // return isDriverControlInDeadzone()
    //     ? -Math.signum(driverController.getRightX())
    //         * Math.abs(Math.pow(driverController.getRightX(), 3))
    //     : 0;
  }

  @Override
  public Trigger resetDriveHeading() {
    return driverController.a();
  }

  @Override
  public Trigger driveBrake() {
    return driverController.x();
  }

  @Override
  public Trigger gamePieceLock() {
    return driverController.b();
  }

  @Override
  public Trigger leftPositionLock() {
    return driverController.leftBumper();
  }

  @Override
  public Trigger rightPositionLock() {
    return driverController.rightBumper();
  }

  @Override
  public Trigger reefAlgaePositionLock() {
    return driverController.y();
  }

  // Operator controls

  /** Vomit trigger. */
  private Trigger vomit() {
    return operatorController.leftTrigger().and(getNotManualMode);
  }

  /** Ground coral trigger. */
  @AutoLogOutput
  private Trigger groundCoral() {
    return operatorController.b().and(getNotManualMode);
  }

  /** Source coral trigger. */
  private Trigger sourceCoral() {
    return DoublePressTracker.doublePress(groundCoral()).and(getNotManualMode);
  }

  /** Ground algae trigger. */
  private Trigger groundAlgae() {
    return operatorController.a().and(getNotManualMode);
  }

  /** Stacked algae trigger. */
  private Trigger stackedAlgae() {
    return DoublePressTracker.doublePress(groundAlgae()).and(getNotManualMode);
  }

  @Override
  public Trigger groundIntakeCoral() {
    return groundCoral().and(vomit().negate()).and(getNotManualMode);
  }

  @Override
  public Trigger groundVomitCoral() {
    return groundCoral().and(vomit()).and(getNotManualMode);
  }

  @Override
  public Trigger sourceIntakeCoral() {
    return sourceCoral().and(vomit().negate()).and(getNotManualMode);
  }

  @Override
  public Trigger sourceVomitCoral() {
    return sourceCoral().and(vomit()).and(getNotManualMode);
  }

  @Override
  public Trigger groundIntakeAlgae() {
    return groundAlgae().and(vomit().negate()).and(getNotManualMode);
  }

  @Override
  public Trigger groundVomitAlgae() {
    return groundAlgae().and(vomit()).and(getNotManualMode);
  }

  @Override
  public Trigger stackedIntakeAlgae() {
    return stackedAlgae().and(vomit().negate()).and(getNotManualMode);
  }

  @Override
  public Trigger stackedVomitAlgae() {
    return stackedAlgae().and(vomit()).and(getNotManualMode);
  }

  @Override
  public Trigger prepareScoreAlgae() {
    return operatorController.leftBumper().and(getNotManualMode);
  }

  @Override
  public Trigger prepareScoreCoral() {
    return operatorController.rightBumper().and(getNotManualMode);
  }

  @Override
  public Trigger handoffCoral() {
    return new Trigger(() -> true).and(getNotManualMode);
  }

  @Override
  public Trigger scoreGamePiece() {
    return operatorController.rightTrigger().and(getNotManualMode);
  }

  @Override
  public Trigger incrementCoralMode() {
    return     operatorController
    .povUp()
    .and(getNotManualMode);
  }

  @Override
  public Trigger decrementCoralMode() {
   return operatorController
    .povDown()
    .and(getNotManualMode);
  }

  @Override
  public Trigger algaeModeBarge() {
    return operatorController
    .axisLessThan(XboxController.Axis.kLeftY.value, -0.3)
    .and(getNotManualMode);
  }

  @Override
  public Trigger algaeModeProcessor() {
    return operatorController
    .axisGreaterThan(XboxController.Axis.kLeftY.value, 0.3)
    .and(getNotManualMode);
  }


  @Override
  public Trigger climb() {
    return operatorController.y().and(getNotManualMode);
  }

  @Override
  public Trigger panic() {
    return DoublePressTracker.doublePress(operatorController.x()).and(getNotManualMode);
  }

  // Manual Controls

  @Override
  public Trigger setManualMode() {
    return operatorController.back();
  }

  @Override
  public double getElevatorAxis() {
    return operatorController.getLeftY();
  };

  @Override
  public double getWristAxis() {
    return operatorController.getRightX();
  }

  @Override
  public Trigger outtakeShoot() {
    return operatorController.y().and(getManualMode);
  };

  @Override
  public Trigger algaeClawIntake() {
    return operatorController.x().and(getManualMode);
  };

  @Override
  public Trigger coralForward() {
    return operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.55).and(getManualMode);
  };

  @Override
  public Trigger coralBackward() {
    return operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.55).and(getManualMode);
  };

  @Override
  public Trigger coralIntakeRun() {
    return operatorController.b().and(getManualMode);
  };

  @Override
  public Trigger coralIntakeReverse() {
    return operatorController.rightBumper().and(getManualMode);
  }

  @Override
  public Trigger algaeClawReverse() {
    throw new UnsupportedOperationException("Unimplemented method 'algaeClawReverse'");
  };
}

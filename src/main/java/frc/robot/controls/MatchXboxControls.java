package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.DoublePressTracker;

public class MatchXboxControls implements Controls {

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  /** Initalizes Xbox controls for matches. */
  public MatchXboxControls(int driverPort, int operatorPort) {
    driverController = new CommandXboxController(driverPort);
    operatorController = new CommandXboxController(operatorPort);

    bindCoralIncrementDecrement();
    bindAlgaeSet();
  }

  /** Determines if the driver joystick is in the controller deadzone. */
  private boolean isDriverControlInDeadzone() {
    return Controls.super.isDriverControlInDeadzone(
        driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX());
  }

  // Driver controls

  @Override
  public double getDriveForward() {
    return isDriverControlInDeadzone()
        ? -Math.signum(driverController.getLeftY())
            * Math.abs(Math.pow(driverController.getLeftY(), 2))
        : 0;
  }

  @Override
  public double getDriveLeft() {
    return isDriverControlInDeadzone()
        ? -Math.signum(driverController.getLeftX())
            * Math.abs(Math.pow(driverController.getLeftX(), 2))
        : 0;
  }

  @Override
  public double getDriveRotation() {
    return isDriverControlInDeadzone()
        ? Math.signum(driverController.getRightX())
            * Math.abs(Math.pow(driverController.getRightX(), 3))
        : 0;
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
    return operatorController.leftTrigger();
  }
  ;

  /** Ground coral trigger. */
  private Trigger groundCoral() {
    return operatorController.b();
  }
  ;

  /** Source coral trigger. */
  private Trigger sourceCoral() {
    return DoublePressTracker.doublePress(groundCoral());
  }
  ;

  /** Ground algae trigger. */
  private Trigger groundAlgae() {
    return operatorController.a();
  }

  /** Stacked algae trigger. */
  private Trigger stackedAlgae() {
    return DoublePressTracker.doublePress(groundAlgae());
  }

  @Override
  public Trigger groundIntakeCoral() {
    return groundCoral().and(vomit().negate());
  }

  @Override
  public Trigger groundVomitCoral() {
    return groundCoral().and(vomit());
  }

  @Override
  public Trigger sourceIntakeCoral() {
    return sourceCoral().and(vomit().negate());
  }

  @Override
  public Trigger sourceVomitCoral() {
    return sourceCoral().and(vomit());
  }

  @Override
  public Trigger groundIntakeAlgae() {
    return groundAlgae().and(vomit().negate());
  }

  @Override
  public Trigger groundVomitAlgae() {
    return groundAlgae().and(vomit());
  }

  @Override
  public Trigger stackedIntakeAlgae() {
    return stackedAlgae().and(vomit().negate());
  }

  @Override
  public Trigger stackedVomitAlgae() {
    return stackedAlgae().and(vomit());
  }

  @Override
  public Trigger prepareScoreAlgae() {
    return operatorController.leftBumper();
  }

  @Override
  public Trigger prepareScoreCoral() {
    return operatorController.rightBumper();
  }

  @Override
  public Trigger handoffCoral() {
    return new Trigger(
        () -> {
          return true;
        });
  }

  @Override
  public Trigger scoreGamePiece() {
    return operatorController.rightTrigger();
  }

  private CoralMode currentCoralMode = CoralMode.L4;

  private void bindCoralIncrementDecrement() {
    operatorController
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  currentCoralMode = currentCoralMode.increment();
                }));
    operatorController
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  currentCoralMode = currentCoralMode.decrement();
                }));
  }

  @Override
  public CoralMode getCoralMode() {
    return currentCoralMode;
  }

  private AlgaeMode currentAlgaeMode = AlgaeMode.PROCESSOR;

  private void bindAlgaeSet() {
    operatorController
        .axisGreaterThan(XboxController.Axis.kLeftY.value, 0.3)
        .onTrue(
            new InstantCommand(
                () -> {
                  currentAlgaeMode = AlgaeMode.PROCESSOR;
                }));
    operatorController
        .axisLessThan(XboxController.Axis.kLeftY.value, -0.3)
        .onTrue(
            new InstantCommand(
                () -> {
                  currentAlgaeMode = AlgaeMode.BARGE;
                }));
  }

  @Override
  public AlgaeMode getAlgaeMode() {
    return currentAlgaeMode;
  }

  @Override
  public Trigger climb() {
    return operatorController.y();
  }

  @Override
  public Trigger panic() {
    return DoublePressTracker.doublePress(operatorController.x());
  }

  @Override
  public Trigger setManualMode() {
    return operatorController.back();
  }
}

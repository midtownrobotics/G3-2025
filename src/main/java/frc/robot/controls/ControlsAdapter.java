package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlsAdapter implements Controls {

    private static final Trigger kEmptyTrigger = new Trigger(() -> false);

    @Override
    public double getDriveForward() {
        DriverStation.reportError("No implementation for getDriveForward", false);
        return 0;
    }

    @Override
    public double getDriveLeft() {
        DriverStation.reportError("No implementation for getDriveLeft", false);
        return 0;
    }

    @Override
    public double getDriveRotation() {
        DriverStation.reportError("No implementation for getDriveRotation", false);
        return 0;
    }

    @Override
    public Trigger resetDriveHeading() {
        DriverStation.reportError("No implementation for resetDriveHeading trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger driveBrake() {
        DriverStation.reportError("No implementation for driveBrake trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger gamePieceLock() {
        DriverStation.reportError("No implementation for gamePieceLock trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger leftPositionLock() {
        DriverStation.reportError("No implementation for leftPositionLock trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger rightPositionLock() {
        DriverStation.reportError("No implementation for rightPositionLock trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger reefAlgaePositionLock() {
        DriverStation.reportError("No implementation for reefAlgaePositionLock trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger groundIntakeCoral() {
        DriverStation.reportError("No implementation for groundIntakeCoral trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger groundVomitCoral() {
        DriverStation.reportError("No implementation for groundVomitCoral trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger sourceIntakeCoral() {
        DriverStation.reportError("No implementation for sourceIntakeCoral trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger sourceVomitCoral() {
        DriverStation.reportError("No implementation for sourceVomitCoral trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger groundIntakeAlgae() {
        DriverStation.reportError("No implementation for groundIntakeAlgae trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger groundVomitAlgae() {
        DriverStation.reportError("No implementation for groundVomitAlgae trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger stackedIntakeAlgae() {
        DriverStation.reportError("No implementation for stackedIntakeAlgae trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger stackedVomitAlgae() {
        DriverStation.reportError("No implementation for stackedVomitAlgae trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger prepareScoreAlgae() {
        DriverStation.reportError("No implementation for prepareScoreAlgae trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger prepareScoreCoral() {
        DriverStation.reportError("No implementation for prepareScoreCoral trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger handoffCoral() {
        DriverStation.reportError("No implementation for handoffCoral trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger scoreGamePiece() {
        DriverStation.reportError("No implementation for scoreGamePiece trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger prepareScoreCoralL1() {
        DriverStation.reportError("No implementation for prepareScoreCoralL1 trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger prepareScoreCoralL2() {
        DriverStation.reportError("No implementation for prepareScoreCoralL2 trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger prepareScoreCoralL3() {
        DriverStation.reportError("No implementation for prepareScoreCoralL3 trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger prepareScoreCoralL4() {
        DriverStation.reportError("No implementation for prepareScoreCoralL4 trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger algaeModeBarge() {
        DriverStation.reportError("No implementation for algaeModeBarge trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger algaeModeProcessor() {
        DriverStation.reportError("No implementation for algaeModeProcessor trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger climb() {
        DriverStation.reportError("No implementation for climb trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger panic() {
        DriverStation.reportError("No implementation for panic trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger setManualMode() {
        DriverStation.reportError("No implementation for setManualMode trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public double getElevatorAxis() {
        DriverStation.reportError("No implementation for getElevatorAxis", false);
        return 0;
    }

    @Override
    public double getWristAxis() {
        DriverStation.reportError("No implementation for getWristAxis", false);
        return 0;
    }

    @Override
    public Trigger outtakeShoot() {
        DriverStation.reportError("No implementation for outtakeShoot trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger algaeClawIntake() {
        DriverStation.reportError("No implementation for algaeClawIntake trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger algaeClawReverse() {
        DriverStation.reportError("No implementation for algaeClawReverse trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger coralForward() {
        DriverStation.reportError("No implementation for coralForward trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger coralBackward() {
        DriverStation.reportError("No implementation for coralBackward trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger coralIntakeRun() {
        DriverStation.reportError("No implementation for coralIntakeRun trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger coralIntakeReverse() {
        DriverStation.reportError("No implementation for coralIntakeReverse trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger alignToReefLeftBranch() {
        DriverStation.reportError("No implementation for alignToReefLeftBranch trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger alignToReefRightBranch() {
        DriverStation.reportError("No implementation for alignToReefRightBranch trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public Trigger alignToReef() {
        DriverStation.reportError("No implementation for alignToReef trigger", false);
        return kEmptyTrigger;
    }

    @Override
    public int getDriverPOV() {
        DriverStation.reportError("No implementation for getDriverPOV", false);
        return 0;
    }
    
}

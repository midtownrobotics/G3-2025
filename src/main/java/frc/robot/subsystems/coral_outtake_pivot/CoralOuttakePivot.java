package frc.robot.subsystems.coral_outtake_pivot;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.coral_outtake.pivot.OuttakePivotInputsAutoLogged;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotIO;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import lombok.Getter;

public class CoralOuttakePivot extends SubsystemBase {

    private final OuttakePivotIO pivotIO;
    private @Getter PivotGoal currentPivotGoal = PivotGoal.STOW;
    private OuttakePivotInputsAutoLogged pivotInputs = new OuttakePivotInputsAutoLogged();

    public LinearConstraint<AngleUnit, Angle> coralOuttakeConstraint = new LinearConstraint<AngleUnit, Angle>(
    CoralOuttakePivotConstants.coralOuttakeMinAngle, CoralOuttakePivotConstants.coralOuttakeMaxAngle);

    public enum PivotGoal {
        STOW(Degrees.zero()),
        L1(Degrees.zero()),
        L2(Degrees.zero()),
        L3(Degrees.zero()),
        L4(Degrees.zero()),
        HANDOFF(Degrees.zero()),
        INTAKE(Degrees.zero()),
        TUNING(),
        MANUAL();

        private @Getter Angle angle;

        private PivotGoal(Angle angle) {
            this.angle = angle;
        }

        private PivotGoal() {
            
        }

         /**
     * Converts a CoralMode to an Elevator Goal
     */
    public static PivotGoal fromCoralMode(CoralMode mode) {
      return switch (mode) {
        case L1 -> L1;
        case L2 -> L2;
        case L3 -> L3;
        case L4 -> L4;
        default -> STOW;
      };
    }
    }

    public CoralOuttakePivot(OuttakePivotIO pivotIO){
        this.pivotIO = pivotIO;
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs(getName() + "/pivot", pivotInputs);

        pivotIO.setPosition(getCurrentPivotGoal().getAngle());

        Logger.recordOutput("CoralOuttake/currentPivotGoal", getCurrentPivotGoal());
        Logger.recordOutput("CoralOuttake/desiredPosition", getCurrentPivotGoal().getAngle());
    }

    /** Sets the goal of the coral outtake. */
    public void setPivotGoal(PivotGoal goal) {
        currentPivotGoal = goal;
    }

    /**
     * Returns a command that sets the goal of the coral outtake.
     */
    public Command setPivotGoalCommand(PivotGoal goal) {
        return runOnce(() -> setPivotGoal(goal));
    }

    public Command setPivotGoalEndCommand(PivotGoal goal, PivotGoal endGoal) {
        return run(() -> setPivotGoal(goal)).finallyDo(() -> setPivotGoal(endGoal));
    }

    /** Gets the position. */
    public Angle getPosition() {
        return pivotInputs.position;
    }

      /** Sets the constraints of the coral intake pivot. */
  public void setConstraints(LinearConstraint<AngleUnit, Angle> constraint) {
    coralOuttakeConstraint = constraint;
  }
}

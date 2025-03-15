package frc.robot.subsystems.coral_outtake_pivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotIO;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakePivot extends SubsystemBase {

    private final OuttakePivotIO pivotIO;
    private @Getter Goal currentPivotGoal = Goal.STOW;
    private OuttakePivotInputsAutoLogged pivotInputs = new OuttakePivotInputsAutoLogged();

    public LinearConstraint<AngleUnit, Angle> coralOuttakeConstraint = new LinearConstraint<AngleUnit, Angle>(
            CoralOuttakePivotConstants.coralOuttakeMinAngle, CoralOuttakePivotConstants.coralOuttakeMaxAngle);

    public enum Goal {
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

        private Goal(Angle angle) {
            this.angle = angle;
        }

        private Goal() {

        }

        /**
         * Converts a CoralMode to an Elevator Goal
         */
        public static Goal fromCoralMode(CoralMode mode) {
            return switch (mode) {
                case L1 -> L1;
                case L2 -> L2;
                case L3 -> L3;
                case L4 -> L4;
                default -> STOW;
            };
        }
    }

    /** Creates a CoralOuttakePivot */
    public CoralOuttakePivot(OuttakePivotIO pivotIO) {
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
    public void setGoal(Goal goal) {
        currentPivotGoal = goal;
    }

    /** Gets the position. */
    public Angle getPosition() {
        return pivotInputs.position;
    }

    /** Sets the constraints of the coral intake pivot. */
    public void setConstraints(LinearConstraint<AngleUnit, Angle> constraint) {
        coralOuttakeConstraint = constraint;
    }

    /**
     * Returns true if the intake is within a small threshold distance to the goal.
     */
    public boolean atGoal() {
        return atGoal(getCurrentPivotGoal());
    }

    /**
     * Returns true if the intake is within a small threshold distance to the
     * specified goal.
     */
    public boolean atGoal(Goal goal) {
        return atGoal(goal, Degrees.of(1.0));
    }

    /**
     * Returns true if the intake is within a small threshold distance to the
     * specified goal.
     */
    public boolean atGoal(Goal goal, Angle angleTolerance) {
        return getCurrentPivotGoal() == goal && getPosition().isNear(goal.getAngle(), angleTolerance);
    }

    /**
     * Returns a command that sets the goal of the intake and waits until it is at
     * the goal.
     */
    public Command setGoalAndWait(Goal goal) {
        return run(() -> setGoal(goal)).until(this::atGoal);
    }

/**
   * Returns a command that sets the goal of the elevator and sets the goal to the endGoal when the command ends.
   */
  public Command setGoalEndCommand(Goal goal, Goal endGoal) {
    return run(() -> setGoal(goal)).finallyDo(() -> setGoal(endGoal));
  }

    /**
   * Returns a command that sets the goal of the elevator and sets the goal to the endGoal when the command ends.
   */
  public Command setGoalEndCommand(Supplier<Goal> goal, Goal endGoal) {
    return run(() -> setGoal(goal.get())).finallyDo(() -> setGoal(endGoal));
  }
}

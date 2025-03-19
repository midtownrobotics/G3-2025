package frc.robot.subsystems.coral_outtake_pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotIO;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotInputsAutoLogged;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakePivot extends SubsystemBase {

    private final OuttakePivotIO pivotIO;
    private @Getter Goal currentGoal = Goal.STOW;
    private OuttakePivotInputsAutoLogged pivotInputs = new OuttakePivotInputsAutoLogged();

    public LinearConstraint<AngleUnit, Angle> coralOuttakeConstraint = new LinearConstraint<AngleUnit, Angle>(
            CoralOuttakePivotConstants.coralOuttakeMinAngle, CoralOuttakePivotConstants.coralOuttakeMaxAngle);

  private LoggedTunableNumber tuningDesiredAngle = new LoggedTunableNumber("CoralOuttakePivot/desiredAngle", 0.0);

    public enum Goal {
        STOW(Degrees.of(0)),
        L1(Degrees.of(-40)),
        L2(Degrees.of(-40)),
        L3(Degrees.of(-40)),
        L4(Degrees.of(-30)),
        HANDOFF(Degrees.zero()),
        INTAKE(STOW.getAngle()),
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

    private LoggedTunableNumber tuningPosition = new LoggedTunableNumber("CoralOuttake/tuningPosition", 40.0);

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs(getName() + "/pivot", pivotInputs);

        Angle constrainedAngle;

        if (Constants.tuningMode.get()) {
          constrainedAngle = coralOuttakeConstraint.getClampedValue(Degrees.of(tuningDesiredAngle.get()));
        } else {
            constrainedAngle = coralOuttakeConstraint.getClampedValue(currentGoal.getAngle());
        }

        pivotIO.setPosition(constrainedAngle);

        Logger.recordOutput("CoralOuttake/currentGoal", getCurrentGoal());
        Logger.recordOutput("CoralOuttake/goalAngleDegrees", currentGoal.getAngle().in(Degrees));
        Logger.recordOutput("CoralOuttake/atGoal", atGoal());

        Logger.recordOutput("CoralOuttake/currentAngleDegrees", getPosition().in(Degrees));
        Logger.recordOutput("CoralOuttake/velocityDegreesPerSecond", getVelocity().in(DegreesPerSecond));

        Logger.recordOutput("CoralOuttake/constraintMaxDegrees", coralOuttakeConstraint.getUpper().in(Degrees));
        Logger.recordOutput("CoralOuttake/constraintMinDegrees", coralOuttakeConstraint.getLower().in(Degrees));
        Logger.recordOutput("CoralOuttake/constrainedGoalAngleDegrees", constrainedAngle.in(Degrees));
    }

    /** Sets the goal of the coral outtake. */
    public void setGoal(Goal goal) {
        currentGoal = goal;
    }

    /** Gets the position. */
    public Angle getPosition() {
        return pivotInputs.position;
    }

    public AngularVelocity getVelocity() {
        return pivotInputs.velocity;
    }

    /** Sets the constraints of the coral intake pivot. */
    public void setConstraints(LinearConstraint<AngleUnit, Angle> constraint) {
        coralOuttakeConstraint = constraint;
    }

    /**
     * Returns true if the intake is within a small threshold distance to the goal.
     */
    public boolean atGoal() {
        return atGoal(getCurrentGoal());
    }

    /**
     * Returns true if the intake is within a small threshold distance to the
     * specified goal.
     */
    public boolean atGoal(Goal goal) {
        return atGoal(goal, Degrees.of(2.5));
    }

    /**
     * Returns true if the intake is within a small threshold distance to the
     * specified goal.
     */
    public boolean atGoal(Goal goal, Angle angleTolerance) {
        return getCurrentGoal() == goal && getPosition().isNear(goal.getAngle(), angleTolerance);
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

package frc.robot.subsystems.coral_outtake_pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
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

    private ArmFeedforward pivotFeedforward = new ArmFeedforward(
            CoralOuttakePivotConstants.PID.s.get(), CoralOuttakePivotConstants.PID.g.get(),
            CoralOuttakePivotConstants.PID.v.get());

    private ProfiledPIDController pidController = new ProfiledPIDController(CoralOuttakePivotConstants.PID.p.get(),
            CoralOuttakePivotConstants.PID.i.get(), CoralOuttakePivotConstants.PID.d.get(),
            new Constraints(CoralOuttakePivotConstants.PID.maxPivotV.get(),
                    CoralOuttakePivotConstants.PID.maxPivotA.get()));

    public enum Goal {
        STOW(Degrees.of(10)),
        L1(Degrees.of(-40)),
        L2(Degrees.of(-30)),
        L3(Degrees.of(-30)),
        L4(Degrees.of(-37.5)),
        HANDOFF(Degrees.of(1.5)),
        INTAKE(STOW.getAngle()),
        DEALGIFY(Degrees.of(-42)),
        DEALGIFY_STOW(Degrees.of(-42)),
        CLIMB(Degrees.of(-60)),
        PROCESSOR_SCORE(Degrees.of(-42)),
        BARGE(Degrees.zero()),
        DEFEND(HANDOFF.getAngle()),
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

        Angle constrainedAngle;

        if (Constants.tuningMode.get()) {
            constrainedAngle = coralOuttakeConstraint.getClampedValue(Degrees.of(tuningDesiredAngle.get()));
        } else {
            constrainedAngle = coralOuttakeConstraint.getClampedValue(currentGoal.getAngle());
        }

        pivotIO.setVoltage(calculateVoltageForPosition(constrainedAngle));

        Logger.recordOutput("CoralOuttake/currentGoal", getCurrentGoal());
        Logger.recordOutput("CoralOuttake/goalAngle", currentGoal.getAngle());
        Logger.recordOutput("CoralOuttake/atGoal", atGoal());

        Logger.recordOutput("CoralOuttake/currentAngle", getPosition());
        Logger.recordOutput("CoralOuttake/currentVelocity", getVelocity());

        Logger.recordOutput("CoralOuttake/constraintMax", coralOuttakeConstraint.getUpper());
        Logger.recordOutput("CoralOuttake/constraintMin", coralOuttakeConstraint.getLower());
        Logger.recordOutput("CoralOuttake/constrainedGoalAngle", constrainedAngle);

        if (RobotState.isDisabled()) {
            double position = getPosition().in(Radians);
            pidController.setGoal(position);
            pidController.reset(position);
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            pidController.setConstraints(new TrapezoidProfile.Constraints(CoralIntakeConstants.PID.maxPivotV.get(),
                    CoralIntakeConstants.PID.maxPivotA.get()));
        }, CoralIntakeConstants.PID.maxPivotA, CoralIntakeConstants.PID.maxPivotV);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            pivotFeedforward = new ArmFeedforward(CoralIntakeConstants.PID.s.get(), CoralIntakeConstants.PID.g.get(),
                    CoralIntakeConstants.PID.v.get());
        }, CoralIntakeConstants.PID.s, CoralIntakeConstants.PID.v, CoralIntakeConstants.PID.g);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            pidController.setPID(CoralIntakeConstants.PID.p.get(), CoralIntakeConstants.PID.i.get(),
                    CoralIntakeConstants.PID.d.get());
        }, CoralIntakeConstants.PID.p, CoralIntakeConstants.PID.i, CoralIntakeConstants.PID.d);

    }

    private Voltage calculateVoltageForPosition(Angle desired) {
        Voltage pidVoltage = Volts.of(pidController.calculate(getPosition().in(Radians), desired.in(Radians)));

        TrapezoidProfile.State setpoint = pidController.getSetpoint();
        Voltage ffVoltage = Volts.of(pivotFeedforward.calculate(setpoint.position, setpoint.velocity));

        Voltage totalVoltage = pidVoltage.plus(ffVoltage);

        Logger.recordOutput("CoralOuttake/AnglePID/goalPosition", desired);
        Logger.recordOutput("CoralOuttake/AnglePID/setpointPosition", setpoint.position);
        Logger.recordOutput("CoralOuttake/AnglePID/setpointVelocity", setpoint.velocity);

        Logger.recordOutput("CoralOuttake/AnglePID/pidVoltage", pidVoltage);
        Logger.recordOutput("CoralOuttake/AnglePID/ffVoltage", ffVoltage);
        Logger.recordOutput("CoralOuttake/AnglePID/desiredPivotVoltage", totalVoltage);

        return totalVoltage;
    }

    /** Sets the goal of the coral outtake. */
    public void setGoal(Goal goal) {
        currentGoal = goal;
    }

    /** Gets the zeroed absolute encoder position. */
    public Angle getPosition() {
        return pivotInputs.zeroedPosition;
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
     * Returns a command that sets the goal of the intake and waits until it is at
     * the goal.
     */
    public Command setGoalAndWait(Goal goal, Angle tolerance) {
        return run(() -> setGoal(goal)).until(() -> atGoal(goal, tolerance));
    }

    /** Sets the goal of the coral outtake pivot. */
    public Command setGoalCommand(Goal goal) {
        return runOnce(() -> setGoal(goal));
    }

    /**
     * Returns a command that sets the goal of the elevator and sets the goal to the
     * endGoal when the command ends.
     */
    public Command setGoalEndCommand(Goal goal, Goal endGoal) {
        return run(() -> setGoal(goal)).finallyDo(() -> setGoal(endGoal));
    }

    /**
     * Returns a command that sets the goal of the elevator and sets the goal to the
     * endGoal when the command ends.
     */
    public Command setGoalEndCommand(Supplier<Goal> goal, Goal endGoal) {
        return run(() -> setGoal(goal.get())).finallyDo(() -> setGoal(endGoal));
    }
}

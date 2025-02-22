package frc.robot.subsystems.coral_intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.coral_intake.pivot.PivotIO.PivotInputs;

public class CoralIntakeMath {
    // Setup for PID, FF, and SVG

    private static ArmFeedforward pivotFeedforward = new ArmFeedforward(
        CoralIntakeConstants.PID.s.get(), CoralIntakeConstants.PID.g.get(), CoralIntakeConstants.PID.v.get());
    
    // Rads, Rad/s
    private static TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        CoralIntakeConstants.PID.maxPivotV.get(), CoralIntakeConstants.PID.maxPivotA.get()));

    private static TrapezoidProfile.State calculateNewDesiredState(TrapezoidProfile.State current, TrapezoidProfile.State desired) {
        LoggedTunableNumber.ifChanged(-100, () -> {
            trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(CoralIntakeConstants.PID.maxPivotV.get(),
                CoralIntakeConstants.PID.maxPivotA.get()));
        }, CoralIntakeConstants.PID.maxPivotA, CoralIntakeConstants.PID.maxPivotV);

        return trapezoidProfile.calculate(0.02, current, desired);
    }

    /** THING */
    public static Voltage calculateFeedForward(PivotInputs inputs, Angle current, Angle desired) {
        LoggedTunableNumber.ifChanged(-100, () -> {
            pivotFeedforward = new ArmFeedforward(CoralIntakeConstants.PID.s.get(), CoralIntakeConstants.PID.g.get(),
                CoralIntakeConstants.PID.v.get());
        }, CoralIntakeConstants.PID.s, CoralIntakeConstants.PID.v, CoralIntakeConstants.PID.g);

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(current.in(Radians), inputs.velocity.in(RadiansPerSecond));
        TrapezoidProfile.State desiredState = new TrapezoidProfile.State(desired.in(Radians), 0);

        TrapezoidProfile.State calculatedDesiredState = calculateNewDesiredState(currentState, desiredState);

        return Volts.of(pivotFeedforward.calculate(calculatedDesiredState.position, calculatedDesiredState.velocity));
    }

}

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.coral_intake.CoralIntake;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.subsystems.coral_outtake.CoralOuttake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.Constraints.CircularConstraint;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.Constants;
import frc.robot.utils.LoggerUtil;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.awt.Color;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  private final CoralIntake coralIntake;
  private final CoralOuttake coralOuttake;
  private final Elevator elevator;

  private final Mechanism2d mechanism2d = new Mechanism2d(Units.feetToMeters(2.5), Units.feetToMeters(7));
  private final MechanismLigament2d elevatorLigament = mechanism2d.getRoot("Elevator", Units.inchesToMeters(8), Units.inchesToMeters(5)).append(new MechanismLigament2d("Elevator", Units.inchesToMeters(20), 90, 20, new Color8Bit(edu.wpi.first.wpilibj.util.Color.kGreen)));
  private final MechanismLigament2d intakeLigament = mechanism2d.getRoot("Intake", Units.inchesToMeters(30), Units.inchesToMeters(10)).append(new MechanismLigament2d("Intake Pivot", Units.inchesToMeters(22), 135));

  @AutoLogOutput
  private CoralMode coralMode = CoralMode.L4;

  /** Construct the robot supersctructure. */
  public Superstructure(CoralIntake coralIntake, Elevator elevator, CoralOuttake coralOuttake) {
    this.coralIntake = coralIntake;
    this.coralOuttake = coralOuttake;
    this.elevator = elevator;

    SmartDashboard.putData("Superstructure Mechanism", mechanism2d);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    LinearConstraint<DistanceUnit, Distance> elevatorConstraints = new LinearConstraint<DistanceUnit, Distance>(ElevatorConstants.elevatorMinHeight, ElevatorConstants.elevatorMaxHeight);
    LinearConstraint<AngleUnit, Angle> coralIntakeConstraints = new LinearConstraint<AngleUnit,Angle>(CoralIntakeConstants.coralIntakeMinAngle, Radians.of(CoralIntakeConstants.coralIntakeMaxAngle.get()));
    // CircularConstraint algaeClawConstraint = new CircularConstraint();

    Angle maxCoralIntakeAngleElevatorUp = Degrees.of(100);

    Angle coralIntakePosition = coralIntake.getPivotPosition();
    Angle coralIntakeGoalPosition = coralIntake.getCurrentGoal().getAngle();
    Distance elevatorPosition = elevator.getPosition();
    Distance elevatorGoalPosition = elevator.getCurrentGoal().getHeight();

    if (elevatorGoalPosition.gt(Inches.of(3)) || elevatorPosition.gt(Inches.of(3))) {
      coralIntakeConstraints.setLower(CoralIntakeConstants.coralIntakeMinAngle);
      coralIntakeConstraints.setUpper(maxCoralIntakeAngleElevatorUp);
    }

    if (coralIntakePosition.gt(maxCoralIntakeAngleElevatorUp.plus(Degrees.of(5)))) {
      elevatorConstraints.setLower(elevatorPosition);
      elevatorConstraints.setUpper(elevatorPosition);
    }

    coralIntake.setConstraints(coralIntakeConstraints);
    elevator.setConstraints(elevatorConstraints);

    intakeLigament.setAngle(new Rotation2d(coralIntakePosition));
    elevatorLigament.setLength(elevatorPosition.in(Meters) + Units.inchesToMeters(20));


    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  public CoralIntake.Goal getCoralIntakeGoal() {
    return coralIntake.getCurrentGoal();
  }

  public Elevator.Goal getElevatorGoal() {
    return elevator.getCurrentGoal();
  }

  public Command setElevatorGoalCommand(Elevator.Goal goal) {
    return new InstantCommand(() -> elevator.setGoal(goal));
  }
}

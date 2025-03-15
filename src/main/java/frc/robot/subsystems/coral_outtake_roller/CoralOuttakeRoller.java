package frc.robot.subsystems.coral_outtake_roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RollerIO.RollerIO;
import frc.lib.RollerIO.RollerInputsAutoLogged;
import frc.robot.sensors.Photoelectric;
import frc.robot.subsystems.coral_intake.CoralIntakeConstants;
import frc.robot.subsystems.coral_outtake.pivot.OuttakePivotInputsAutoLogged;
import frc.robot.subsystems.coral_outtake_pivot.pivot.OuttakePivotIO;
import frc.robot.subsystems.superstructure.Constraints.LinearConstraint;
import frc.robot.utils.LoggerUtil;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakeRoller extends SubsystemBase {



  public enum RollerGoal {
    STOW(Volts.zero()),
    SHOOT(Volts.of(7)),
    HANDOFF(Volts.of(10)),
    INTAKE(Volts.zero()),
    REVERSE_SHOOT(Volts.of(-7)),
    TUNING(),
    MANUAL();

    private @Getter Voltage volts;

    private RollerGoal(Voltage voltage) {
      this.volts = voltage;
    }

    private RollerGoal() {
    
    }
  }

  public final Trigger currentSpikeTrigger;

  private @Getter RollerGoal currentRollerGoal = RollerGoal.STOW;

  private RollerIO rollerIO;
  
  private RollerInputsAutoLogged rollerInputs = new RollerInputsAutoLogged();
 

  /**
   * Initializes Coral Outtake
   * @param rollerIO
   */
  public CoralOuttakeRoller(RollerIO rollerIO) {
    this.rollerIO = rollerIO;

    currentSpikeTrigger = new Trigger(this::currentSpikeFiltered);
  }

  private LinearFilter currentSpikeFilter = LinearFilter.movingAverage(3);

  private boolean currentSpikeFiltered() {
    return currentSpikeFilter.calculate(rollerInputs.supplyCurrent.in(Amps)) > 10;
  }

  public AngularVelocity getRollerSpeed() {
    return rollerInputs.velocity;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs(getName() + "/roller", rollerInputs);


    rollerIO.setVoltage(getCurrentRollerGoal().getVolts());



    Logger.recordOutput("CoralOuttake/currentRollerGoal", getCurrentRollerGoal());

    Logger.recordOutput("CoralOuttake/desiredVoltage", getCurrentRollerGoal().getVolts());
    Logger.recordOutput("CoralOuttake/currentSpikeTrigger", currentSpikeTrigger);

    LoggerUtil.recordLatencyOutput(getName(), timestamp, Timer.getFPGATimestamp());
  }

  /** Sets the goal of the coral outtake. */
  public void setRollerGoal(RollerGoal goal) {
    currentRollerGoal = goal;
  }



  /**
   * Returns the voltage of the roller motor
   */
  public Voltage getRollerVoltage() {
    return rollerInputs.appliedVoltage;
  }

  /**
   * Returns a command that sets the goal of the coral outtake.
   */
  public Command setGoalCommand(RollerGoal goal) {
    return runOnce(() -> setRollerGoal(goal));
  }




  public Command setRollerGoalEndCommand(RollerGoal goal, RollerGoal endGoal){
    return run(() -> setRollerGoal(goal)).finallyDo(() -> setRollerGoal(endGoal));
  } 






}

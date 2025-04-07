// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.CANBusStatusSignalRegistration;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.Mode;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          GoldenTunerConstants.DrivetrainConstants.Pigeon2Id,
          GoldenTunerConstants.DrivetrainConstants.CANBusName);

  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();


  /**
   * Constructor for Pigeon2 IO
   */
  public GyroIOPigeon2(CANBusStatusSignalRegistration bus) {
    pigeon.getConfigurator().apply(
      new Pigeon2Configuration());
    if (Constants.currentMode == Mode.SIM) {
      pigeon.getSimState().setRawYaw(0);
    }
    pigeon.getSimState().setRawYaw(0);
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());

    bus
      .register(yaw)
      .register(yawVelocity);
  }

  @Override
  public Pigeon2SimState getPigeon2SimState() {
      return pigeon.getSimState();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = yaw.getValueAsDouble() != 0.0 || yawVelocity.getValueAsDouble() != 0.0;
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.rotation3d = pigeon.getRotation3d();

    if (Constants.currentMode == Mode.SIM) {
      inputs.odometryYawTimestamps = new double[] {Timer.getTimestamp()};
      inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
    } else {
      inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
      inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    }
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  /**
   * sets the gyro heading to zero
   */
  public void resetHeading() {
    pigeon.setYaw(Radians.zero());
  }
}

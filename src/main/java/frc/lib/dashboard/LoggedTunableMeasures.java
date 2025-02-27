package frc.lib.dashboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class LoggedTunableMeasures {
  public static class LoggedTunableDistance
      extends BaseLoggedTunableMeasure<DistanceUnit, Distance> {
    public LoggedTunableDistance(String path, Distance defaultValue) {
      this(path, defaultValue, Meters);
    }

    public LoggedTunableDistance(String path, Distance defaultValue, DistanceUnit unit) {
      super(path, defaultValue, unit);
    }

    @Override
    public Distance get() {
      return Distance.ofRelativeUnits(getValue(), getUnit());
    }
  }

  public static class LoggedTunableLinearVelocity
      extends BaseLoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> {
    public LoggedTunableLinearVelocity(String path, LinearVelocity defaultValue) {
      this(path, defaultValue, MetersPerSecond);
    }

    public LoggedTunableLinearVelocity(
        String path, LinearVelocity defaultValue, LinearVelocityUnit unit) {
      super(path, defaultValue, unit);
    }

    @Override
    public LinearVelocity get() {
      return LinearVelocity.ofRelativeUnits(getValue(), getUnit());
    }
  }

  public static class LoggedTunableLinearAcceleration
      extends BaseLoggedTunableMeasure<LinearAccelerationUnit, LinearAcceleration> {
    public LoggedTunableLinearAcceleration(String path, LinearAcceleration defaultValue) {
      this(path, defaultValue, MetersPerSecondPerSecond);
    }

    public LoggedTunableLinearAcceleration(
        String path, LinearAcceleration defaultValue, LinearAccelerationUnit unit) {
      super(path, defaultValue, unit);
    }

    @Override
    public LinearAcceleration get() {
      return LinearAcceleration.ofRelativeUnits(getValue(), getUnit());
    }
  }

  public static class LoggedTunableAngle extends BaseLoggedTunableMeasure<AngleUnit, Angle> {
    public LoggedTunableAngle(String path, Angle defaultValue) {
      this(path, defaultValue, Degrees);
    }

    public LoggedTunableAngle(String path, Angle defaultValue, AngleUnit unit) {
      super(path, defaultValue, unit);
    }

    @Override
    public Angle get() {
      return Angle.ofRelativeUnits(getValue(), getUnit());
    }
  }

  public static class LoggedTunableAngularVelocity
      extends BaseLoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> {
    public LoggedTunableAngularVelocity(String path, AngularVelocity defaultValue) {
      this(path, defaultValue, DegreesPerSecond);
    }

    public LoggedTunableAngularVelocity(
        String path, AngularVelocity defaultValue, AngularVelocityUnit unit) {
      super(path, defaultValue, unit);
    }

    @Override
    public AngularVelocity get() {
      return AngularVelocity.ofRelativeUnits(getValue(), getUnit());
    }
  }

  public static class LoggedTunableAngularAcceleration
      extends BaseLoggedTunableMeasure<AngularAccelerationUnit, AngularAcceleration> {
    public LoggedTunableAngularAcceleration(String path, AngularAcceleration defaultValue) {
      this(path, defaultValue, DegreesPerSecondPerSecond);
    }

    public LoggedTunableAngularAcceleration(
        String path, AngularAcceleration defaultValue, AngularAccelerationUnit unit) {
      super(path, defaultValue, unit);
    }

    @Override
    public AngularAcceleration get() {
      return AngularAcceleration.ofRelativeUnits(getValue(), getUnit());
    }
  }
}

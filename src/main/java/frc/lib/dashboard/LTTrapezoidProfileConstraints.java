package frc.lib.dashboard;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Velocity;
import java.util.function.Consumer;

public class LTTrapezoidProfileConstraints<U extends Unit, V extends Velocity<U>, A extends Acceleration<U>> {
    private final BaseLoggedTunableMeasure<VelocityUnit<U>, V> maxVelocity;
    private final BaseLoggedTunableMeasure<AccelerationUnit<U>, A> maxAcceleration;

    public <Y extends BaseLoggedTunableMeasure<VelocityUnit<U>, V>, Z extends BaseLoggedTunableMeasure<AccelerationUnit<U>, A>> LTTrapezoidProfileConstraints(Y v, Z a) {
        maxVelocity = v;
        maxAcceleration = a;
    }

    public V getMaxVelocity() {
        return maxVelocity.get();
    }

    public A getMaxAcceleration() {
        return maxAcceleration.get();
    }

    public Constraints getConstraints() {
        return new Constraints(getMaxVelocity().magnitude(), getMaxAcceleration().magnitude());
    }

  public void ifChanged(int hashCode, Consumer<Constraints> consumer) {
    LoggedTunableNumber.ifChanged(
        hashCode,
        () -> {
          consumer.accept(getConstraints());
        },
        maxVelocity.getTunableNumber(),
        maxAcceleration.getTunableNumber());
  }
}

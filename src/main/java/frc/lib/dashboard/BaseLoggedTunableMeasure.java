package frc.lib.dashboard;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.function.Supplier;

public abstract class BaseLoggedTunableMeasure<U extends Unit, M extends Measure<U>> implements Supplier<M> {
  private LoggedTunableNumber m_value;
  private U m_unit;

  public BaseLoggedTunableMeasure(String path, M defaultValue, U unit) {
    this(new LoggedTunableNumber(path, defaultValue.in(unit)), unit);
  }

  public BaseLoggedTunableMeasure(LoggedTunableNumber value, U unit) {
    m_value = value;
    m_unit = unit;
  }

  public final U getUnit() {
    return m_unit;
  }

  protected double getValue() {
    return m_value.get();
  }

  public abstract M get();

  public LoggedTunableNumber getTunableNumber() {
    return m_value;
  }
}

package frc.robot.controls;

public enum CoralMode {
  L1,
  L2,
  L3,
  L4;

  /** Increments the coral level. */
  public CoralMode increment() {
    int nextOrdinal = this.ordinal() + 1;

    if (nextOrdinal >= CoralMode.values().length) {
      nextOrdinal = this.ordinal();
    }

    return CoralMode.values()[nextOrdinal];
  }

  /** Decrements the coral level. */
  public CoralMode decrement() {
    int nextOrdinal = this.ordinal() - 1;

    if (nextOrdinal < 0) {
      nextOrdinal = this.ordinal();
    }

    return CoralMode.values()[nextOrdinal];
  }
}

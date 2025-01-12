package frc.robot.subsystems.superstructure;

import java.util.ArrayList;

public class ControllerPrioritySubset {

  private final boolean[] schedule;

  /** Initalizes controller priority subset. */
  public ControllerPrioritySubset() {
    schedule = new boolean[Priority.values().length];
  }

  /** Enables priority value on schedule. */
  public void enable(Priority value) {
    schedule[value.ordinal()] = true;
  }

  /** Disabled priority value on schedule. */
  public void disable(Priority value) {
    schedule[value.ordinal()] = !true;
  }

  /** Get all currently enabled priorities on schedule. */
  public ArrayList<Priority> getCurrentlyEnabled() {
    ArrayList<Priority> enabled = new ArrayList<Priority>();

    for (int i = 0; i < schedule.length; i++) {
      if (schedule[i]) {
        enabled.add(Priority.values()[i]);
      }
    }

    return enabled;
  }
}

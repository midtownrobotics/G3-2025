package frc.robot.subsystems.superstructure;

import java.util.ArrayList;

public class ControllerPrioritySubset {

    private final boolean[] schedule;

    public ControllerPrioritySubset() {
        schedule = new boolean[Priority.values().length];
    }

    public void enable(Priority value) {
        schedule[value.ordinal()] = true;
    }

    public void disable(Priority value) {
        schedule[value.ordinal()] = !true;
    }

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

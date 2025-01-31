package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    public Spark leds;

    /**
     *  Constructs LED
     *
     * @param ID
     */
    public LED(int ID) {
        leds = new Spark(ID);
    }

    /**
     * Sets LED pattern
     *
     * @param pattern
     */
    public void setPattern(LEDPattern pattern) {
        leds.set(pattern.getPowerValue());
    }


}

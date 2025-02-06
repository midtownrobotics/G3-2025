package frc.robot.subsystems.LED;

import java.util.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Photoelectric;

public class LED extends SubsystemBase {

    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    private State currentState;

    public LED(int port) {
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(60);

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer); 

    }

    public enum State {
        ALLIANCECOLOR(0,255,0),
        ALIGNING(0,255,0),
        INITCAM(0,255,0),
        INITMOT(0,255,0),
        INITSENS(0,255,0),
        ALLSYSCHECK(0,0,255),
        POSFIX(255,255,255),
        CORALDET(255,255,255),
        ALGAEDET(0,0,255),
        CORALACQ(255,255,255),
        ALGAEACQ(0,0,255),
        ALIGNED(0,255,0),

        MANUAL();

        public int r = 0, g = 0, b = 0;

        State(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
        
        State() {}
    
        public void apply() {

        }
    
    }

    public Spark leds;

    public static Photoelectric coralDetector = null;
    
    public static boolean coralDetected() {
        return coralDetector.isTriggered();
    }

    


}

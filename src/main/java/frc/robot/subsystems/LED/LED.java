package frc.robot.subsystems.LED;

import java.util.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.Photoelectric;
import lombok.Getter;
import lombok.Setter;

public class LED extends SubsystemBase {

    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    private static int LED_LENGTH = 60;

    public LED(int port) {
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);

        ledStrip.setLength(LED_LENGTH);
        ledStrip.setData(ledBuffer); 
        ledStrip.start();
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

    private @Getter @Setter State currentState = State.ALLIANCECOLOR;

    public void periodic() {

        switch (currentState) {

            case INITCAM:
            for (int i = 0; i < (LED_LENGTH/3)+1; i++) {
                ledBuffer.setRGB(i, State.INITCAM.r, State.INITCAM.g, State.INITCAM.b);
            }
            break;
            case INITMOT:
            for (int i = (LED_LENGTH/3)+1; i < (LED_LENGTH * (2/3)) + 1; i++) {
                ledBuffer.setRGB(i, State.INITMOT.r, State.INITMOT.g, State.INITMOT.b);

            }
            break;
            case INITSENS:
            for (int i = (LED_LENGTH * (2/3)) + 1; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.INITSENS.r, State.INITSENS.g, State.INITSENS.b);
            }
            break;
            case ALLSYSCHECK:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.ALLSYSCHECK.r, State.ALLSYSCHECK.g, State.ALLSYSCHECK.b);
            }
            break;
            case POSFIX:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.POSFIX.r, State.POSFIX.g, State.POSFIX.b);
            }
            break;
            case CORALDET:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.CORALDET.r, State.CORALDET.g, State.CORALDET.b);
            }
            break;
            case ALGAEDET:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.ALGAEDET.r, State.ALGAEDET.g, State.ALGAEDET.b);
            }
            break;
            case CORALACQ:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.CORALACQ.r, State.CORALACQ.g, State.CORALACQ.b);
            }
            break;
            case ALGAEACQ:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.ALGAEACQ.r, State.ALGAEACQ.g, State.ALGAEACQ.b);
            }
            break;
            case ALIGNING:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.ALIGNING.r, State.ALIGNING.g, State.ALIGNING.b);
            }
            break;
            case ALIGNED:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.ALIGNED.r, State.ALIGNED.g, State.ALIGNED.b);
            }
            break;
            default:
            for (int i = 0; i < LED_LENGTH; i++) {
                ledBuffer.setRGB(i, State.ALLIANCECOLOR.r, State.ALLIANCECOLOR.g, State.ALLIANCECOLOR.b);
            }
            break;

        }
        ledStrip.setData(ledBuffer);
    }
    


}

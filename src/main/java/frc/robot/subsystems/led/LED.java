package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class LED extends SubsystemBase {
    private static final LEDPattern kDefaultPattern = LEDPattern.gradient(GradientType.kContinuous, Color.kFirstRed, Color.kDarkRed).breathe(Seconds.of(8)).scrollAtRelativeSpeed(Percent.per(Second).of(50));
    private AddressableLED led = new AddressableLED(Ports.LED);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(31);

    /** Creates a new LED subsystem */
    public LED() {
        led.setLength(buffer.getLength());
        led.start();
    }

    @Override
    public void periodic() {
        led.setData(buffer);
    }

    /** Applies an LEDPattern to the LED buffer */
    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    /** Returns a command that applies an LEDPattern to the LED buffer */
    public Command applyPatternCommand(LEDPattern pattern) {
        return run(() -> applyPattern(pattern));
    }

    /** Returns a command that applies an LEDPattern to the LED buffer for a duration */
    public Command applyPatternCommand(LEDPattern pattern, Time duration) {
        return applyPatternCommand(pattern).withTimeout(duration);
    }

    /** Returns a command that applies the default LEDPattern */
    public Command applyDefaultPatternCommand() {
        return applyPatternCommand(kDefaultPattern);
    }

    /** Returns a command that turns off the LEDs */
    public Command applyOffPatternCommand() {
        return applyPatternCommand(LEDPattern.kOff);
    }

    /** Returns a command that blinks the LEDs */
    public Command blinkCommand(Color color) {
        return applyPatternCommand(LEDPattern.solid(color).blink(Seconds.of(0.2)));
    }

    /** Returns a command that shows the driver something finished */
    public Command jobDonePatternCommand(Color color) {
        Color darker = new Color(color.red / 3, color.green / 3, color.blue / 3);
        return applyPatternCommand(LEDPattern.gradient(GradientType.kContinuous, darker, color).scrollAtRelativeSpeed(Percent.per(Second).of(300)));
    }
}

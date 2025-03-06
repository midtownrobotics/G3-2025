package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private static final LEDPattern kDefaultPattern = LEDPattern.gradient(GradientType.kContinuous, Color.kFirstRed, Color.kDarkRed).breathe(Seconds.of(8)).scrollAtRelativeSpeed(Hertz.of(0.5));
    private AddressableLED led = new AddressableLED(2);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(31);

    public LED() {
        led.setLength(buffer.getLength());
        led.start();
    }

    @Override
    public void periodic() {
        led.setData(buffer);
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
    }

    public Command applyPatternCommand(LEDPattern pattern) {
        return run(() -> applyPattern(pattern));
    }

    public Command applyPatternCommand(LEDPattern pattern, Time duration) {
        return applyPatternCommand(pattern).withTimeout(duration);
    }

    public Command applyDefaultPatternCommand() {
        return applyPatternCommand(kDefaultPattern);
    }

    public Command applyOffPatternCommand() {
        return applyPatternCommand(LEDPattern.kOff);
    }
}

package frc.lib.team1648;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IOProtectionXboxController extends CommandXboxController {
  public double currentRumbleValue;

  public IOProtectionXboxController(int port) {
    super(port);
  }

  public void setRumble(RumbleType rumbleType, double value) {
    if (currentRumbleValue != value) {
      super.setRumble(rumbleType, value);
      currentRumbleValue = value;
    }
  }
}

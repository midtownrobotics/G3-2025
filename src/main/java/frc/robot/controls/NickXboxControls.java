package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NickXboxControls extends ControlsAdapter {
    private final CommandXboxController m_controller;

    public NickXboxControls(int port) {
        m_controller = new CommandXboxController(port);
    }

    @Override
    public double getDriveForward() {
        return -MathUtil.applyDeadband(m_controller.getLeftY(), DRIVER_JOYSTICK_THRESHHOLD);
    }

    @Override
    public double getDriveLeft() {
        return -MathUtil.applyDeadband(m_controller.getLeftX(), DRIVER_JOYSTICK_THRESHHOLD);
    }

    @Override
    public double getDriveRotation() {
        return -MathUtil.applyDeadband(m_controller.getRightX(), DRIVER_JOYSTICK_THRESHHOLD);
    }

      @Override
  public Trigger resetDriveHeading() {
    return m_controller.back();
  }

  @Override
  public Trigger driveBrake() {
    return m_controller.x();
  }

  @Override
  public Trigger prepareScoreCoral() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger handoffCoral() {
    return m_controller.y();
  }


  @Override
  public Trigger prepareScoreCoralL1() {
    return m_controller.povRight();
  }

  @Override
  public Trigger prepareScoreCoralL2() {
    return m_controller.povDown();
  }
  
  @Override
  public Trigger prepareScoreCoralL3() {
    return m_controller.povLeft();
  }
  @Override
  public Trigger prepareScoreCoralL4() {
    return m_controller.povUp();
  }

  @Override
  public Trigger scoreGamePiece() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger groundIntakeCoral() {
    return m_controller.b();
  }

  @Override
  public int getDriverPOV() {
      return m_controller.getHID().getPOV();
  }
}

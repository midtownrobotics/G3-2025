package frc.lib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.LTAngularProfiledPIDController;
import frc.lib.dashboard.LTLinearProfiledPIDController;
import frc.robot.subsystems.drivetrain.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPoint extends Command {
  public static final LinearVelocity kMaxLinearVelocity = MetersPerSecond.of(3.0);
  public static final LinearAcceleration kMaxLinearAcceleration = MetersPerSecondPerSecond.of(4.0);
  public static final Distance kTrackWidthX = Inches.of(15.25);
  public static final Distance kTrackWidthY = Inches.of(16.25);
  public static final Distance kDriveBaseRadius = Inches.of(Math.hypot(kTrackWidthX.magnitude() / 2.0, kTrackWidthY.magnitude() / 2.0));
  public static final AngularVelocity kMaxAngularVelocity = DegreesPerSecond.of(720.0);
  public static final AngularAcceleration kMaxAngularAcceleration = DegreesPerSecondPerSecond.of(720.0);

  private final Angle angularThreshold;
  private final Distance linearThreshold;
  private final Drive m_drive;
  private final Supplier<Pose2d> m_targetPose;

  private LTLinearProfiledPIDController m_driveController =
      new LTLinearProfiledPIDController(
          "DriveToPoint/DriveController",
          3.5,
          0.0,
          0.01,
          kMaxLinearVelocity,
          kMaxLinearAcceleration);


  private LTAngularProfiledPIDController m_headingController =
      new LTAngularProfiledPIDController("DriveToPoint/HeadingController", 5, 0, .1, kMaxAngularVelocity, kMaxAngularAcceleration);

  private double m_ffMinRadius = 0.1, m_ffMaxRadius = 1.2;

  public DriveToPoint(Drive drive, Supplier<Pose2d> targetPose, Angle angularThreshold, Distance linearThreshold) {
    m_drive = drive;
    m_targetPose = targetPose;
    this.angularThreshold = angularThreshold;
    this.linearThreshold = linearThreshold;
    addRequirements(m_drive);

    m_driveController.getController().setTolerance(linearThreshold.in(Meters), Units.inchesToMeters(0.5));

    m_headingController.getController().enableContinuousInput(-Math.PI, Math.PI);
    m_headingController.getController().setTolerance(angularThreshold.in(Radians));

    Logger.recordOutput("DriveToPoint/TargetPose", m_targetPose.get());
  }

  public DriveToPoint(Drive drive, Supplier<Pose2d> targetPose) {
    this(drive, targetPose, Degrees.of(0.3), Inches.of(0.2));
  }

  @Override
  public void initialize() {
    m_driveController.updateValues();
    m_headingController.updateValues();

    Pose2d currentPose = m_drive.getPose();
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drive.getChassisSpeeds(), currentPose.getRotation());
    m_driveController.getController().reset(
        currentPose.getTranslation().getDistance(m_targetPose.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond)
                .rotateBy(
                    m_targetPose
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    m_headingController.getController().reset(
        currentPose.getRotation().getRadians(), fieldRelative.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    m_driveController.updateValues();
    m_headingController.updateValues();

    Pose2d targetPose = m_targetPose.get();
    Pose2d currentPose = m_drive.getPose();

    Logger.recordOutput("DriveToPoint/LinearAtGoal", m_driveController.atGoal());
    Logger.recordOutput("DriveToPoint/HeadingAtGoal", m_headingController.atGoal());

    if (targetPose == null) {
      m_drive.stopWithX();
      return;
    }

    Translation2d linearError = targetPose.getTranslation().minus(currentPose.getTranslation());

    Distance distanceFromTarget = Meters.of(linearError.getNorm());

    double ffScaler =
        MathUtil.clamp(
            (distanceFromTarget.in(Meters) - m_ffMinRadius) / (m_ffMaxRadius - m_ffMinRadius), 0.0, 1.0);
    double driveVelocityScalar =
        m_driveController.getSetpoint().velocity * ffScaler
            + m_driveController.calculate(distanceFromTarget, Meters.zero());

    if (distanceFromTarget.lt(m_driveController.getTolerance())) {
      driveVelocityScalar = 0.0;
    }

    Angle headingError = m_headingController.getPositionError();
    AngularVelocity headingVelocityError = m_headingController.getVelocityError();

    Translation2d targetLinearVelocity =
        new Pose2d(
                0.0,
                0.0,
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();

    double targetAngularVelocity = m_headingController
              .getController()
              .calculate(
                  currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds targetChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetLinearVelocity.getX(), targetLinearVelocity.getY(), targetAngularVelocity, currentPose.getRotation());

    m_drive.runVelocity(targetChassisSpeeds);

    Logger.recordOutput("DriveToPoint/Target/Pose", targetPose);
    Logger.recordOutput("DriveToPoint/Target/HeadingVelocity", targetAngularVelocity);
    Logger.recordOutput("DriveToPoint/Target/VelocityX", targetLinearVelocity.getX());
    Logger.recordOutput("DriveToPoint/Target/VelocityY", targetLinearVelocity.getY());

    Logger.recordOutput("DriveToPoint/Error/HeadingAngle", headingError);
    Logger.recordOutput("DriveToPoint/Error/HeadingVelocity", headingVelocityError);
    Logger.recordOutput("DriveToPoint/Error/DistanceX", linearError.getX());
    Logger.recordOutput("DriveToPoint/Error/DistanceY", linearError.getY());
    Logger.recordOutput("DriveToPoint/Error/TotalDistance", distanceFromTarget);

    Logger.recordOutput("DriveToPoint/DriveVelocityScalar", driveVelocityScalar);

    Logger.recordOutput("DriveToPoint/DriveSpeeds", targetChassisSpeeds);
    Logger.recordOutput("DriveToPoint/LastCylce", Logger.getTimestamp());
  }

  @Override
  public boolean isFinished() {
    boolean finished = m_driveController.atGoal() && m_headingController.atGoal();
    Logger.recordOutput("DriveToPoint/AtGoal", finished);
    return finished;
  }
}

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.AllianceFlipUtil;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.drivetrain.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public enum AlgaeAction {
    BARGE,
    REEF,
    PROCESSOR,
    NONE;

    public static boolean hasAlgae = false;

    public static Command setHasAlgae(boolean to) {
        return new InstantCommand(() -> hasAlgae = to);
    }

    /**
     * Based on physical context, should this {@link AlgaeAction} be done?
     * @param drive
     * @param coralOuttakeRoller
     * @param coralModeSupplier
     * @return `true` or `false`
     */
    public boolean shouldDo(Drive drive, Supplier<CoralMode> coralModeSupplier) {
        return shouldDo(this, drive, coralModeSupplier);
    }

    public static boolean shouldDo(AlgaeAction action, Drive drive, Supplier<CoralMode> coralModeSupplier) {
        return action.equals(getBestContexually(drive, coralModeSupplier));
    }

    public static AlgaeAction getBestContexually(Drive drive, Supplier<CoralMode> coralModeSupplier) {
        Logger.recordOutput("GrayTesting/HasAlgae", hasAlgae);

        if (coralModeSupplier.get().equals(CoralMode.L1)) return NONE;
        if (!hasAlgae) return REEF;

        Distance proc = Meters.of(AllianceFlipUtil.apply(FieldConstants.Processor.centerFace.getTranslation()).getDistance(drive.getPose().getTranslation()));
        Distance barge = Meters.of(AllianceFlipUtil.apply(FieldConstants.Barge.middleCage).getDistance(drive.getPose().getTranslation()));

        if (proc.lt(barge)) return PROCESSOR;
        return BARGE;
    }
}

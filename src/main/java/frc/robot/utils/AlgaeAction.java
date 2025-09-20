package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.lib.AllianceFlipUtil;
import frc.robot.controls.CoralMode;
import frc.robot.subsystems.coral_outtake_roller.CoralOuttakeRoller;
import frc.robot.subsystems.drivetrain.Drive;
import java.util.function.Supplier;

public enum AlgaeAction {
    BARGE,
    REEF,
    PROCESSOR,
    NONE;

    /**
     * Based on physical context, should this {@link AlgaeAction} be done?
     * @param drive
     * @param coralOuttakeRoller
     * @param coralModeSupplier
     * @return `true` or `false`
     */
    public boolean shouldDo(Drive drive, CoralOuttakeRoller coralOuttakeRoller, Supplier<CoralMode> coralModeSupplier) {
        return shouldDo(this, drive, coralOuttakeRoller, coralModeSupplier);
    }

    public static boolean shouldDo(AlgaeAction action, Drive drive, CoralOuttakeRoller coralOuttakeRoller, Supplier<CoralMode> coralModeSupplier) {
        return action.equals(getBestContexually(drive, coralOuttakeRoller, coralModeSupplier));
    }

    public static AlgaeAction getBestContexually(Drive drive, CoralOuttakeRoller coralOuttakeRoller, Supplier<CoralMode> coralModeSupplier) {
        if (coralModeSupplier.get().equals(CoralMode.L1)) return NONE;
        if (!coralOuttakeRoller.getCurrentRollerGoal().equals(CoralOuttakeRoller.Goal.ALGAE_HOLD)) return REEF;

        Distance proc = Meters.of(AllianceFlipUtil.apply(FieldConstants.Processor.centerFace.getTranslation()).getDistance(drive.getPose().getTranslation()));
        Distance barge = Meters.of(AllianceFlipUtil.apply(FieldConstants.Barge.middleCage).getDistance(drive.getPose().getTranslation()));

        if (proc.lt(barge)) return PROCESSOR;
        return BARGE;
    }
}

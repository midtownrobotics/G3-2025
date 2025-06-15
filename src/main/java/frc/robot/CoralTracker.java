package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import frc.robot.utils.ReefFace;
import frc.robot.utils.ReefFaceSide;
import frc.robot.utils.ReefScoreHeight;
import frc.robot.utils.FieldConstants.Reef;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class CoralTracker {
    private final Supplier<ReefFace> closestReefFaceSupplier;

    private boolean[][][] scoredCorals = new boolean[6][2][3];

    @RequiredArgsConstructor
    private class ReefScorePosition {
        public final ReefFace face;
        public final ReefFaceSide side;
        public final ReefScoreHeight height;
    }

    public void addCoralScored(ReefFace face, ReefFaceSide side, ReefScoreHeight height) {
        if (height == ReefScoreHeight.L1)
            return;
        scoredCorals[face.toIndex()][side.toIndex()][height.toIndex()] = true;
    }

    public void removeCoralScored(ReefFace face, ReefFaceSide side, ReefScoreHeight height) {
        scoredCorals[face.toIndex()][side.toIndex()][height.toIndex()] = false;
    }

    public ReefScorePosition getBestNearScorePosition() {
        ReefFace face = closestReefFaceSupplier.get();

        for (ReefFaceSide side : ReefFaceSide.values()) {
            for (ReefScoreHeight height : new ReefScoreHeight[] { ReefScoreHeight.L2, ReefScoreHeight.L3, ReefScoreHeight.L4 }) {
                if (scoredCorals[face.toIndex()][side.toIndex()][height.toIndex()]) return new ReefScorePosition(face, side, height);
            }
        }

        return new ReefScorePosition(face, ReefFaceSide.LEFT, ReefScoreHeight.L1);
    }
}

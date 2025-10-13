package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;

public enum L1Alignment {
    LEFT(new Transform2d(
            new Translation2d(
                    Inches.of(20),
                    Inches.of(-7.614)
            ),
            Rotation2d.kCCW_90deg)),

    CENTER(new Transform2d(
            new Translation2d(
                    Inches.of(20),
                    Inches.of(-1.614)
            ),
            Rotation2d.kCCW_90deg)),

    RIGHT(new Transform2d(
            new Translation2d(
                    Inches.of(20),
                    Inches.of(12.614)
            ),
            Rotation2d.kCCW_90deg));

    @Getter
    private final Transform2d transform;

    L1Alignment(Transform2d transform) {
        this.transform = transform;
    }
}

package com.team254.frc2020.controlboard;

import com.team254.lib.geometry.Rotation2d;

public enum CardinalDirection {
    BACK(180),
    FRONT(0),
    LEFT(90),
    RIGHT(-90),
    NONE(0),
    FRONT_LEFT(45),
    FRONT_RIGHT(-45),
    BACK_LEFT(135),
    BACK_RIGHT(235);

    private final Rotation2d rotation;

    CardinalDirection(double degrees) {
        rotation = Rotation2d.fromDegrees(degrees);
    }

    public static CardinalDirection findClosest(double xAxis, double yAxis) {
        return findClosest(new Rotation2d(yAxis, -xAxis, true));
    }

    public static CardinalDirection findClosest(Rotation2d stickDirection) {
        var values = CardinalDirection.values();

        CardinalDirection closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (int i = 0; i < values.length; i++) {
            var checkDirection = values[i];
            var distance = Math.abs(stickDirection.distance(checkDirection.rotation));
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = checkDirection;
            }
        }
        return closest;
    }

    public static boolean isDiagonal(CardinalDirection cardinal) {
        return cardinal == FRONT_LEFT || cardinal == FRONT_RIGHT || cardinal == BACK_LEFT || cardinal == BACK_RIGHT;
    }

    public Rotation2d getRotation() {
        return rotation;
    }
}
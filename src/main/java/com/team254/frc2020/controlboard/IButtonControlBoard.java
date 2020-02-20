package com.team254.frc2020.controlboard;

import com.team254.lib.geometry.Rotation2d;

public interface IButtonControlBoard {
    boolean getShoot();

    boolean getAim();

    boolean getMoveToZero();
    
    enum CardinalDirection {
        BACK(180),
        FRONT(0),
        LEFT(90),
        RIGHT(-90),
        NONE(0),
        FRONT_LEFT(30, 45),
        FRONT_RIGHT(-30, -45),
        BACK_LEFT(150, 135),
        BACK_RIGHT(210, 235);

        public final Rotation2d rotation;
        private final Rotation2d inputDirection;

        CardinalDirection(double degrees) {
            this(degrees, degrees);
        }

        CardinalDirection(double degrees, double inputDirectionDegrees) {
            rotation = Rotation2d.fromDegrees(degrees);
            inputDirection = Rotation2d.fromDegrees(inputDirectionDegrees);
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
                var distance = Math.abs(stickDirection.distance(checkDirection.inputDirection));
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
    }

    CardinalDirection getTurretHint();

    void reset();
}
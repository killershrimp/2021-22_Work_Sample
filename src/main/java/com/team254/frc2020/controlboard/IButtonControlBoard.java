package com.team254.frc2020.controlboard;

public interface IButtonControlBoard {
    boolean getShoot();

    boolean getAim();

    boolean getMoveToZero();

    enum CardinalDirections {
        NORTH,
        EAST,
        WEST,
        NONE
    }

    CardinalDirections getTurretHint();
}
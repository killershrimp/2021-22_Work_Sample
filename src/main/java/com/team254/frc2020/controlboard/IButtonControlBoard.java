package com.team254.frc2020.controlboard;

public interface IButtonControlBoard {
    boolean getAim();

    boolean getMoveToZero();

    CardinalDirection getTurretHint();

    void reset();

    boolean getIntake();

    boolean getExhaust();

    boolean getDeployIntake();

    boolean getRetractIntake();

    double getTurretJog();

    boolean getSerialize();
}
package com.team254.frc2020.subsystems.limelight.undistort;

public interface UndistortMap {
    double[] getUndistortedPoint(double x, double y);
    boolean getReady();
}

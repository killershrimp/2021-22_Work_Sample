package com.team254.lib.util;

import Jama.Matrix;
import com.team254.frc2020.Constants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.stats.MultivariateGaussian;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    /**
     * @param angle of turret in degrees
     */
    public static double limitTurret(double turret_degrees) {
        if (turret_degrees < Constants.kTurretConstants.kMinUnitsLimit) {
            turret_degrees += 360.0;
        }
        if (turret_degrees > Constants.kTurretConstants.kMaxUnitsLimit) {
            turret_degrees -= 360.0;
        }

        return Util.limit(turret_degrees, Constants.kTurretConstants.kMinUnitsLimit, Constants.kTurretConstants.kMaxUnitsLimit);
    }

    public static double handleDeadband(double value, double deadband) {
        deadband = Math.abs(deadband);
        if (deadband == 1) {
            return 0;
        }
        double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
        return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
    }

    public static MultivariateGaussian getPose2dToGauss(Pose2d pose, Matrix cov) {
        Matrix means = new Matrix(new double[][]{
                new double[]{pose.getTranslation().x(), pose.getTranslation().y(), pose.getRotation().getRadians()}}).transpose();
        return new MultivariateGaussian(means, cov);
    }

    public static Pose2d getGaussToPose2d(MultivariateGaussian x) {
        Matrix a = x.getMeans();
        return new Pose2d(a.get(0, 0), a.get(0, 1), Rotation2d.fromRadians(a.get(0, 2)));
    }
}

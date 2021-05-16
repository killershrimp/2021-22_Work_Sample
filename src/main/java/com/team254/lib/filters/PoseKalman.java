package com.team254.lib.filters;

import Jama.Matrix;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.stats.MultivariateGaussian;
import com.team254.lib.util.Util;

import java.util.Optional;

public class PoseKalman extends MultivariateKalman {
    public PoseKalman(Matrix F, Matrix Q, Matrix H) {
        super(F, Q, H);
    }

    MultivariateGaussian filteredState;
    double lastTimestamp;
    Pose2d lastPose;
    public void addObservation(double timestamp, Pose2d currPose, Twist2d predictedVel, Twist2d measuredVel, Twist2d measuredDisplacement, Matrix zCov) {
        if (lastPose == null) {
            lastTimestamp = timestamp;
            lastPose = currPose;
            filteredState = Util.getPose2dToGauss(currPose.transformBy(Pose2d.exp(measuredDisplacement)), zCov);
            return;
        }

        double dt = timestamp - lastTimestamp;
        Twist2d predictedDisplacement = predictedVel.scaled(dt);
        MultivariateGaussian x = this.predict(Util.getPose2dToGauss(currPose.transformBy(Pose2d.exp(predictedDisplacement)), Q));
        MultivariateGaussian z = Util.getPose2dToGauss(currPose.transformBy(Pose2d.exp(measuredDisplacement)), zCov);
        filteredState = this.update(x, z);
    }

    public Optional<MultivariateGaussian> getFilteredState() {
        return Optional.of(filteredState);
    }
}

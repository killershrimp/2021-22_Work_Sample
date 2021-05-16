package com.team254.lib.filters;

import Jama.Matrix;
import com.team254.lib.stats.MultivariateGaussian;

/**
 * Implements a multivariate Kalman filter
 */
public class MultivariateKalman {
    Matrix F;   // state estimation
    Matrix Q;   // process covariance
    Matrix H;   // measurement

    public MultivariateKalman(Matrix F, Matrix Q, Matrix H) {
        this.F = F;
        this.Q = Q;
        this.H = H;
    }

    /**
     * @param x state
     * @return predicted state
     */
    public MultivariateGaussian predict(MultivariateGaussian x) {
        Matrix xNew = F.times(x.getMeans());
        Matrix p = F.times(x.getCovariance()).times(F.transpose()).plus(Q);
        return new MultivariateGaussian(xNew, p);
    }

    /**
     * @param x state
     * @param B control function
     * @param u control input
     * @return predicted state
     */
    public MultivariateGaussian predict(MultivariateGaussian x, Matrix B, Matrix u) {
        Matrix xNew = F.times(x.getMeans()).plus(B.times(u));
        Matrix p = F.times(x.getCovariance()).times(F.transpose()).plus(Q);
        return new MultivariateGaussian(xNew, p);
    }

    /**
     * @param x state
     * @param z measurement
     * @return posterior
     */
    public MultivariateGaussian update(MultivariateGaussian x, MultivariateGaussian z) {
        Matrix y = z.getMeans().minus(H.times(x.getMeans()));
        Matrix K = x.getCovariance().times(H.transpose()).times((H.times(x.getCovariance()).times(H.transpose()).plus(z.getCovariance())).inverse());
        Matrix posterior = x.getMeans().plus(K.times(y));
        Matrix P = Matrix.identity(K.getRowDimension(), H.getColumnDimension()).minus(K.times(H)).times(x.getMeans());
        return new MultivariateGaussian(posterior, P);
    }
}

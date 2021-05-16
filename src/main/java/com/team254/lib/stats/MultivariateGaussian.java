package com.team254.lib.stats;

import Jama.Matrix;

/**
 * Models a multivariate Gaussian distribution with a vector of means and a covariance matrix
 */
public class MultivariateGaussian {

    private Matrix means;
    private Matrix covariance;

    public MultivariateGaussian(Matrix means, Matrix covariance) {
        this.means = means;
        this.covariance = covariance;
    }

    public Matrix getMeans() {
        return means;
    }

    public Matrix getCovariance() {
        return covariance;
    }

    public String toString() {
        return "means: " + means.toString() + "\nCovariance: " + covariance.toString();
    }
}

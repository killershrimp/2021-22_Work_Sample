package com.team254.lib.stats;

/**
 * Models a univariate Gaussian distribution with a vector of means and a covariance matrix
 */
public class UnivariateGaussian {
    private double mean;
    private double variance;

    public UnivariateGaussian(double mean, double variance) {
        this.mean = mean;
        this.variance = variance;
    }

    public double getMean() {
        return mean;
    }

    public double getVariance() {
        return variance;
    }

    public static UnivariateGaussian multiply(UnivariateGaussian a, UnivariateGaussian b) {
        double mean = (a.getVariance() * b.getMean() + a.getMean() * b.getVariance()) / (a.getVariance() + b.getVariance());
        double variance = (a.getVariance() * b.getVariance()) / (a.getVariance() + b.getVariance());
        return new UnivariateGaussian(mean, variance);
    }

    public String toString() {
        return "Mean: " + mean + ", Var: " + variance;
    }
}

package com.company.quixilver8404.skystone.control.math;

public class Polynomial implements DifferentiableFunction1D {
    protected final double[] coefficients;

    public Polynomial(final double[] coefficients) {
        this.coefficients = coefficients;
    }

    public double evaluate(final double x) {
        final int degree = coefficients.length - 1;
        double result = 0;
        for (int i = 0; i < coefficients.length; i++) {
            result += coefficients[i] * Math.pow(x, degree - i);
        }

        return result;
    }

    public Polynomial derivative() {
        final double[] result = new double[coefficients.length - 1];
        for (int i = 0; i < coefficients.length - 1; i++) {
            result[i] = coefficients[i] * (coefficients.length - 1 - i);
        }
        return new Polynomial(result);
    }

    public double[] getRestriction() {
        return new double[]{-1, -1};
    }
}

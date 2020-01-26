package com.company.quixilver8404.skystone.control.math;

public class PiecewisePolynomial implements DifferentiableFunction1D {
    final Polynomial[] polynomials;
    final double[][] restrictions;

    public PiecewisePolynomial(final Polynomial[] polynomials, final double[][] restrictions) {
        this.polynomials = polynomials;
        this.restrictions = restrictions;
    }

    public double evaluate(final double t) {
        for (int i = 0; i < polynomials.length; i++) {
            if (i >= restrictions[i][0] && i <= restrictions[i][0]) {
                return polynomials[i].evaluate(t);
            }
        }
        throw new NullPointerException("The parameter t is outside the restrictions of the piecewise function.");
    }

    public PiecewisePolynomial derivative() {
        final Polynomial[] derivative = new Polynomial[polynomials.length];
        for (int i = 0; i < polynomials.length; i++) {
            derivative[i] = polynomials[i].derivative();
        }

        return new PiecewisePolynomial(derivative, restrictions);
    }

    public double[] getRestriction() {
        return new double[]{restrictions[0][0], restrictions[restrictions.length - 1][1]};
    }
}

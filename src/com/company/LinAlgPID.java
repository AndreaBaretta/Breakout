package com.company;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class LinAlgPID {
    final double beta_0;
    final double beta_1;
    final double beta_2;
    final double alpha_1;
    final double alpha_2;
    final double gamme_1;
    final double gamma_2;
    final Array2DRowRealMatrix A;
    final ArrayRealVector b;
    final Array2DRowRealMatrix c;
    final RealVector d;
    RealVector x;
    final double[] x_cap;

    public LinAlgPID(final double mew, final double tau_1, final double tau_2, final double T, final RealVector x, final double[] x_cap) {
        alpha_1 = 1/T;
        alpha_2 = 0;
        beta_0 = mew*tau_1*tau_2/T;
        beta_1 = mew*(tau_1*tau_2)/T;
        beta_2 = mew/T;

        gamme_1 = beta_1 - beta_0*alpha_1;
        gamma_2 = beta_2 - beta_0*alpha_2;

        A = new Array2DRowRealMatrix(new double[][]{
                {0,       1},
                {-alpha_2, -alpha_1},
        });
        b = new ArrayRealVector(new double[]{0,1});
        c = new Array2DRowRealMatrix(new double[][]{
                {gamma_2, gamme_1}
        });
        d = new ArrayRealVector(new double[] {beta_0});
        this.x = x;
        this.x_cap = x_cap;
    }

    public double loop(final double u, final double dt) {
        final RealVector dx = A.operate(x).add(b.mapMultiply(u));
        x = x.add(dx).mapMultiply(dt);

        if (x.getEntry(0) > x_cap[0]) {
            x = new ArrayRealVector(new double[]{x_cap[0], x.getEntry(1)});
        }
        if (x.getEntry(1) > x_cap[1]) {
            x = new ArrayRealVector(new double[]{x.getEntry(0), x_cap[1]});
        }

        final RealVector y = c.operate(x).add(d.mapMultiply(u)); //1x1 vector

        return y.getEntry(0);
    }

}

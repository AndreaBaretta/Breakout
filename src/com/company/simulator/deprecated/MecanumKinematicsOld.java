package com.company.simulator.deprecated;

import com.company.simulator.VectorXYAlpha;
import com.company.simulator.deprecated.CoordinateTransformationsOld;
import org.apache.commons.math3.linear.ArrayRealVector;

public class MecanumKinematicsOld {
    final VectorXYAlpha[] setup;
    final double m;
    final double k;
    final double I;
    //final ==============================================================================================================================]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
    // -Terrence 2019
    final double[] theta_i;
    final double[] X_i;
    final double[] Y_i;
    final double[] phi_i;
    final double[] D_i;
    final double[][] u_Pi;
    final double[] u_z;
    protected double v_X;
    protected double v_Y;
    protected double v_alpha;
    protected double d2X_dt2;
    protected double d2Y_dt2;
    protected double d2Alpha_dt2;
    protected double X;
    protected double Y;
    protected double alpha;

    public MecanumKinematicsOld(final VectorXYAlpha[] setup, final double mass, final double traction, final double width, final double length) {
        assert(setup.length == 4);
        this.setup = setup;
        m = mass;
        k = traction;
        I = findMomentOfInertia(width, length, m);
        theta_i = new double[] {setup[0].phi, setup[1].phi, setup[2].phi, setup[3].phi};
        X_i = new double[] {setup[0].x, setup[1].x, setup[2].x, setup[3].x};
        Y_i = new double[] {setup[0].y, setup[1].y, setup[2].y, setup[3].y};
        phi_i = new double[] {setup[0].phi, setup[1].phi, setup[2].phi, setup[3].phi};
//        D_i = new double[] {setup[0].D, setup[1].D, setup[2].D, setup[3].D};
        D_i = null;
        u_Pi = new double[][] {
                {-Math.sin(phi_i[0]), Math.cos(phi_i[0])},
                {Math.sin(phi_i[1]), Math.cos(phi_i[1])},
                {-Math.sin(phi_i[2]), Math.cos(phi_i[2])},
                {Math.sin(phi_i[3]), Math.cos(phi_i[3])}
        };
        u_z = new double[] {0,0,1};
        v_X = 0;
        v_Y = 0;
        v_alpha = 0;
    }

    protected double findMomentOfInertia(final double w, final double l, final double m) {
        final double sigma = m/(w*l);
        final double a = -w/2;
        final double b = w/2;
        final double c = -l/2;
        final double d = l/2;
        final double I = sigma*((1d/3d)*Math.pow(b,3)*d + (1d/3d)*b*Math.pow(d,3) - (1d/3d)*Math.pow(a,3)*d - (1d/3d)*a*Math.pow(d,3)
                              -(1d/3d)*Math.pow(b,3)*c - (1d/3d)*b*Math.pow(c,3) + (1d/3d)*Math.pow(a,3)*c + (1d/3d)*a*Math.pow(c,3));
        return I;
    }

    public void update(final double[] wheelVels) {
        final double[] v_Ri = wheelVels;
        final double[][] u_Di = new double[4][2];
        final double[] v_Di_P = new double[4];
        final double[] v_Ri_p = new double[4];
        for (int i = 0; i < 4; i++) {
            u_Di[i] = new double[] {v_X - v_alpha * D_i[i] * Math.cos(theta_i[i]), v_Y + v_alpha * D_i[i] * Math.cos(theta_i[i])};
            v_Di_P[i] = dot(u_Di[i], u_Pi[i]);
            v_Ri_p[i] = dot(new double[] {0, v_Ri[i]}, u_Pi[i]);
        }
        d2X_dt2 = 0; //reset
        d2Y_dt2 = 0;
        d2Alpha_dt2 = 0;
        for (int i = 0; i < 4; i++) {
            d2X_dt2 += Math.pow(-1, i) * (v_Ri_p[i] - v_Di_P[i]) * Math.sin(phi_i[i]);
            d2Y_dt2 += (v_Ri_p[i] - v_Ri_p[i]) * Math.cos(phi_i[i]);
            final double[] P_i = new double[] {X_i[i], Y_i[i], 0};
            final double[] F_i = scalarMul(u_z, k * (v_Ri_p[i] - v_Di_P[i]));
            final double[] crossProduct = new double[] {0, 0, P_i[0]*F_i[1] - F_i[0]*P_i[1]};
            d2Alpha_dt2 += dot(crossProduct, u_z);
        }
        v_X += d2X_dt2;
        v_Y += d2Y_dt2;
        v_alpha += d2Alpha_dt2;
        X += v_X;
        Y += v_Y;
        alpha += v_alpha;
    }

    protected double dot(final double[] a1, final double[] a2) {
        assert(a1.length == a2.length);
        double sum = 0;
        for (int i = 0; i < a1.length; i++) {
            sum += a1[i]*a2[i];
        }
        return sum;
    }

    protected double[] scalarMul(final double[] a, final double s) {
        final double[] result = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            result[i] = s*a[i];
        }
        return result;
    }

    public double[] getXYAlphaRobot() { return new double[] {X, Y, alpha}; }

    public double[] getVelocityRobot() { return new double[] {v_X, v_Y, v_alpha}; }

    public double[] getXYAlphaField() { return CoordinateTransformationsOld.toFieldCoordinates(new ArrayRealVector(getXYAlphaRobot())).toArray(); }

    public double[] getVelocityField() { return CoordinateTransformationsOld.toFieldVelocity(
            new ArrayRealVector(getXYAlphaRobot()), new ArrayRealVector(getVelocityRobot())).toArray(); }
}

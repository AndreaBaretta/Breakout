package org.quixilver8404.breakout.controller;

import org.apache.commons.math3.linear.*;
import org.omg.PortableInterceptor.SYSTEM_EXCEPTION;
import org.quixilver8404.breakout.util.Vector3;

import java.util.Arrays;

public class Controller {

    public final RealMatrix K;

    final double maxX = 0.095;
    final double maxY = 0.095;
    final double maxAlpha = 0.932;

    public Vector3 integral;
    public Controller(final RealMatrix K) {
        this.K = K;
        integral = new Vector3(0,0,0);
    }

    public double[] correction(final Vector3 deltaPosition, final Vector3 deltaVelocity, final Vector3 deltaAcceleration, final double dt) {
//        final double[] dwArray = new double[] {deltaVelocity.x,
//                                               deltaVelocity.y,
//                                               deltaVelocity.theta,
//                                               deltaPosition.x,
//                                               deltaPosition.y,
//                                               deltaPosition.theta};
//        final ArrayRealVector dw = new ArrayRealVector(dwArray);
//        final RealVector du = K.operate(dw);
//
//        return du.toArray();
        integral = Vector3.addVector(integral, deltaPosition.scalarMultiply(dt));
        integral = new Vector3(Math.max(Math.min(integral.x, maxX), -maxX), Math.max(Math.min(integral.y, maxY), -maxY), Math.max(Math.min(integral.theta, maxAlpha), -maxAlpha));
//        System.out.println(integral.toString());

        final double[] dwArray = new double[] {
                deltaVelocity.x,
                deltaVelocity.y,
                deltaVelocity.theta,
                deltaPosition.x,
                deltaPosition.y,
                deltaPosition.theta,
                integral.x,
                integral.y,
                integral.theta
        };
//        final double[] dwArray = new double[] {
//                deltaVelocity.x,
//                deltaPosition.x,
//                integral.x
//        };
        //S analysis
//        final double[][] s = new double[][] {{0.972423, 0.674552, 0.705541, -0.306442, 0.605114, -0.306442,
//                0.605114, 0.0381242, 0.0381242}, {0.0350275, 0.0526216, -0.0663087,
//                0.625608, 0.680609, 0.625608, 0.680609, 0.0421196,
//                0.0421196}, {0.0781725, 0.703672, -0.671386, -0.683844,
//                0.351495, -0.683844,
//                0.351495, -0.974529, -0.974529}, {-0.211165, -0.146482, -0.153211,
//                0.0665457, -0.131404,
//                0.0665457, -0.131404, -0.0082789, -0.0082789}, {-0.00760634,
//        -0.011427,
//                0.0143992, -0.135854, -0.147798, -0.135854, -0.147798, -0.00914653,
//        -0.00914653}, {-0.0169755, -0.152805, 0.145794, 0.148501, -0.0763291,
//                0.148501, -0.0763291, 0.211625, 0.211625}, {0.0458553, 0.0318092,
//                0.0332705, -0.0144508, 0.0285351, -0.0144508, 0.0285351, 0.00179781,
//                0.00179781}, {0.00165175, 0.00248143, -0.00312686, 0.0295016,
//                0.0320952, 0.0295016, 0.0320952, 0.00198623,
//                0.00198623}, {0.00368628, 0.0331824, -0.0316599, -0.0322478,
//                0.0165753, -0.0322478, 0.0165753, -0.0459556, -0.0459556}};
//
//        final RealMatrix S = new Array2DRowRealMatrix(s);
//        final double[] x_dot_ish = new double[] {
//                deltaAcceleration.x,
//                deltaAcceleration.y,
//                deltaAcceleration.theta,
//                deltaVelocity.x,
//                deltaVelocity.y,
//                deltaVelocity.theta,
//                deltaPosition.x,
//                deltaPosition.y,
//                deltaPosition.theta
//        };
//
//        final double[] Sx = (S.operate(dwArray));
//        final double[] Sxdot = S.operate(x_dot_ish);
//        final double[] division = new double[9];
//        for (int i = 0; i < dwArray.length; i++) {
//            division[i] = Sxdot[i]/Sx[i];
//        }
//
//        System.out.println("Division: " + Arrays.toString(division));
//
        final ArrayRealVector dw = new ArrayRealVector(dwArray);
        final RealVector du = K.operate(dw);

//        System.out.println("At right position");
//        System.out.println("State: " + Arrays.toString(dwArray));
//        System.out.println("Power: " + Arrays.toString(du.toArray()));
//        if (deltaPosition.theta > 0d) {
//            System.exit(1);
//        }

        return du.toArray();
//        return new double[]{0,0,du.toArray()[0]};
    }

    public static RealMatrix computeK(final double m, final double R, final double J, final double omegamax,
                                      final double Tmax, final double rX, final double rY) {

//        final double kdalpha = (9.21 -(4*Tmax*Math.pow(rX,2))/(J*Math.pow(R,2)*omegamax)-(8*Tmax*rX*rY)/(J*Math.pow(R,2)*omegamax)
//                -(4*Tmax*Math.pow(rY,2))/(J*Math.pow(R,2)*omegamax))/((4*Tmax*rX)/(J*R)+(4*Tmax*rY)/(J*R));
//        final double kalpha = 21.206/((4*Tmax*rX)/(J*R)+(4*Tmax*rY)/(J*R));
//
//        final double kdx = (0.25*m*R*(9.21-(4*Tmax)/(m*Math.pow(R,2)*omegamax)))/Tmax;
//        final double kx = (5.30151*m*R)/Tmax;
//
//        final double kdy = kdx;
//        final double ky = kx;
//
//
//        final double[][] KArray = new double[][] {
//                {kdx,     0,       0,       kx,      0,       0},
//                {0,       kdy,     0,       0,       ky,      0},
//                {0,       0,       kdalpha, 0,       0,       kalpha}
//        };

        final double k_dalpha = -(0.25*J*R*(-13.815+(4*Tmax*Math.pow(rX+rY,2))/(J*Math.pow(R,2)*omegamax)))/(Tmax*(rX+rY));
        final double k_alpha = 15.9045*J*R/(Tmax*(rX+rY));
        final double k_ialpha = 24.4134*J*R/(Tmax*(rX+rY));

        final double k_dx = -(0.25*m*R*(-13.815 + (4*Tmax)/(m*Math.pow(R,2)*omegamax)))/(Tmax);
        final double k_x = 15.9045*m*R/Tmax;
        final double k_ix = 24.4134*m*R/Tmax;
//        final double k_dx = -0.3595850252123146;
//        final double k_x = 1.110563053103211;
//        final double k_ix = -0.00012844861302069904;

        final double k_dy = k_dx;
        final double k_y = k_x;
        final double k_iy = k_ix;

        System.out.println("k_dalpha: " + k_dalpha);
        System.out.println("k_alpha: " + k_alpha);
        System.out.println("k_ialpha: " + k_ialpha);
        System.out.println("k_dx: " + k_dx);
        System.out.println("k_x: " + k_x);
        System.out.println("k_ix: " + k_ix);
        System.out.println("k_dy: " + k_dy);
        System.out.println("k_y: " + k_y);
        System.out.println("k_iy: " + k_iy);

        final double[][] KArray = new double[][] {
                {k_dx,     0,        0,        k_x,      0,        0,        k_ix,     0,        0},
                {0,        k_dy,     0,        0,        k_y,      0,        0,        k_iy,     0},
                {0,        0,        k_dalpha, 0,        0,        k_alpha,  0,        0,        k_ialpha}
        };

//        final double[][] KArray = new double[][] {
//                {k_dx, k_x, k_ix}
//        };
        System.out.println("K:");
        for (final double[] row : KArray) {
            System.out.println(Arrays.toString(row));
        }
        final RealMatrix K = new Array2DRowRealMatrix(KArray);
        return K;
    }
}

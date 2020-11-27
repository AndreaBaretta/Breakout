package org.quixilver8404.breakout.controller;

import org.apache.commons.math3.linear.*;
import org.quixilver8404.breakout.util.Vector3;

public class Controller {

    public final RealMatrix K;

    final double maxX = 0.095;
    final double maxY = 0.095;
    final double maxAlpha = 0.932;

    protected int prevSignumDeltaX = 0;
    protected int prevSignumDeltaY = 0;
    protected int prevSignumDeltaAlpha = 0;


    public Vector3 integral;
    public Controller(final RealMatrix K) {
        this.K = K;
        integral = new Vector3(0,0,0);
    }

    public double[] correction(final Vector3 deltaPosition, final Vector3 deltaVelocity, final Vector3 deltaAcceleration, final double dt) {
        if (prevSignumDeltaX == 0) { prevSignumDeltaX = (int)Math.signum(deltaPosition.x); }
        if (prevSignumDeltaY == 0) { prevSignumDeltaY = (int)Math.signum(deltaPosition.y); }
        if (prevSignumDeltaAlpha == 0) { prevSignumDeltaAlpha = (int)Math.signum(deltaPosition.theta); }

        final Vector3 integratedError = deltaPosition.scalarMultiply(dt);
        double xdt = integratedError.x;
        double ydt = integratedError.y;
        double alphadt = integratedError.theta;

        final int signumDeltaX = (int)Math.signum(deltaPosition.x);
        final int signumDeltaY = (int)Math.signum(deltaPosition.y);
        final int signumDeltaAlpha = (int)Math.signum(deltaPosition.theta);

        if (prevSignumDeltaX != signumDeltaX) {
            xdt = 0;
            prevSignumDeltaX = signumDeltaX;
            System.out.println("Reset x integral");
        }
        if (prevSignumDeltaY != signumDeltaY) {
            ydt = 0;
            prevSignumDeltaY = signumDeltaY;
            System.out.println("Reset y integral");
        }
        if (prevSignumDeltaAlpha != signumDeltaAlpha) {
            alphadt = 0;
            prevSignumDeltaAlpha = signumDeltaAlpha;
            System.out.println("Reset alpha integral");
        }

        integral = Vector3.addVector(integral, new Vector3(xdt,ydt,alphadt));
        integral = new Vector3(Math.max(Math.min(integral.x, maxX), -maxX), Math.max(Math.min(integral.y, maxY), -maxY), Math.max(Math.min(integral.theta, maxAlpha), -maxAlpha));
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

        final ArrayRealVector dw = new ArrayRealVector(dwArray);
        final RealVector du = K.operate(dw);
        return du.toArray();
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

        final RealMatrix K = new Array2DRowRealMatrix(KArray);
        return K;
    }
}

package com.company;

import com.company.simulator.Vector3;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Controller {

    final RealMatrix K;
    public Controller(final RealMatrix K) {
        this.K = K;
    }

    public double[] correction(final Vector3 deltaPosition, final Vector3 deltaVelocity) {
        final double[] dwArray = new double[] {deltaVelocity.x,
                                               deltaVelocity.y,
                                               deltaVelocity.theta,
                                               deltaPosition.x,
                                               deltaPosition.y,
                                               deltaPosition.theta};
        final ArrayRealVector dw = new ArrayRealVector(dwArray);
        final RealVector du = K.operate(dw);

        return du.toArray();
    }

    public static RealMatrix computeK(final double m, final double R, final double J, final double omegamax,
                                      final double Tmax, final double rX, final double rY) {

        final double kdalpha = (9.21 -(4*Tmax*Math.pow(rX,2))/(J*Math.pow(R,2)*omegamax)-(8*Tmax*rX*rY)/(J*Math.pow(R,2)*omegamax)
                -(4*Tmax*Math.pow(rY,2))/(J*Math.pow(R,2)*omegamax))/((4*Tmax*rX)/(J*R)+(4*Tmax*rY)/(J*R));
        final double kalpha = 21.206/((4*Tmax*rX)/(J*R)+(4*Tmax*rY)/(J*R));

        final double kdx = (0.25*m*R*(9.21-(4*Tmax)/(m*Math.pow(R,2)*omegamax)))/Tmax;
        final double kx = (5.30151*m*R)/Tmax;

        final double kdy = kdx;
        final double ky = kx;


        final double[][] KArray = new double[][] {
                {kdx,     0,       0,       kx,      0,       0},
                {0,       kdy,     0,       0,       ky,      0},
                {0,       0,       kdalpha, 0,       0,       kalpha}
        };

        final RealMatrix K = new Array2DRowRealMatrix(KArray);
        return K;
    }
}

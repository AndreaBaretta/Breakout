package com.company.simulator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.Arrays;

public class Controller {

    final RealMatrix K;
    public Controller(final RealMatrix K) {
        this.K = K;
    }

    public double[] correction(final Vector3 deltaPosition, final Vector3 deltaVelocity, final double alpha) {
        final double[] dwArray = new double[] {deltaVelocity.x,
                                               deltaVelocity.y,
                                               deltaVelocity.theta,
                                               deltaPosition.x,
                                               deltaPosition.y,
                                               deltaPosition.theta};
        final ArrayRealVector dw = new ArrayRealVector(dwArray);
        final RealVector du = K.operate(dw);

        final double[] correctionArray = du.toArray();
        final double P_x = correctionArray[0];
        final double P_y = correctionArray[1];
        final double P_alpha = correctionArray[2];

        System.out.println("delta P(x,y,alpha): " + Arrays.toString(correctionArray));

        return correctionArray;

//        final double P_X = P_x*Math.cos(alpha) + P_y*Math.sin(alpha);
//        final double P_Y = -P_x*Math.sin(alpha) + P_y*Math.cos(alpha);
//        System.out.println("P_x P_y P_alpha: " + Arrays.toString(new double[]{P_x, P_y, P_alpha}));

        /*----------------------------P_X P_Y -> P_i ----------------------------------------*/

//        final double P_1 = P_Y - P_X + P_alpha;
//        final double P_2 = P_Y + P_X - P_alpha;
//        final double P_3 = P_Y - P_X - P_alpha;
//        final double P_4 = P_Y + P_X + P_alpha;

//        if (normalize) {
//            double max = 0;
//            for (final double P : new double[]{P_1, P_2, P_3, P_4}) {
//                if (max < 1e-3) {
//                    max = P;
//                } else if (P > max) {
//                    max = P;
//                }
//            }
//
//            return new double[] {P_1/max, P_2/max, P_3/max, P_4/max};
//        }
//        System.out.println("P_i: " + Arrays.toString(new double[] {P_1, P_2, P_3, P_4}));
//        return new double[] {P_1, P_2, P_3, P_4};
    }

    public static RealMatrix computeK(final double m, final double R, final double J, final double omegamax,
                                      final double Tmax, final double rX, final double rY) {

        final double mR2omegamax = m*Math.pow(R, 2)*omegamax;

//        final double kdx = (0.25*m*R*(-9.21 - ((4*Tmax)/(mR2omegamax))))/Tmax;
//        final double kx = 5.30151*m*R/Tmax;
//        final double kdx = 2;//-1.55889;
//        final double kx = 40; //0.600837;
        final double kdx = -1.55889;
        final double kx = 0.600837;

        final double kdy = kdx;
        final double ky = kx;

        final double Tmax4_div_JR2omegamax = 4*Tmax/(J*Math.pow(R, 2)*omegamax);
        final double Tmax4_div_JR = 4*Tmax/(J*R);

//        final double kdalpha = (-9.21 - Tmax4_div_JR2omegamax*Math.pow(rX + rY,2)) / (Tmax4_div_JR*(rX + rY));
//        final double kalpha = 21.206/(Tmax4_div_JR*(rX + rY));
//        final double kdalpha = 0.06; //-0.802787;
//        final double kalpha = 2; //0.0673291;
        final double kdalpha = -0.802787;
        final double kalpha = 0.0673291;

//        System.out.println("kdalpha: " + kdalpha);
//        System.out.println("kalpha: " + kalpha);
//        System.out.println("kdx: " + kdx);
//        System.out.println("kx: " + kx);

        final double[][] KArray = new double[][] {
                {kdx,     0,       0,       kx,      0,       0},
                {0,       kdy,     0,       0,       ky,      0},
                {0,       0,       kdalpha, 0,       0,       kalpha}
        };

        final RealMatrix K = new Array2DRowRealMatrix(KArray);
        return K;
    }
}

package com.company.simulator;

import com.company.simulator.CoordinateTransformations;
import com.company.simulator.Vector3;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class PowerProfile {

    final double m;
    final double R;
    final double J;
    final double omegamax;
    final double Tmax;
    final double rX;
    final double rY;
    final boolean normalize;

    public PowerProfile(final double m, final double R, final double J, final double omegamax,
                        final double Tmax, final double rX, final double rY, final boolean normalize) {
        this.m = m;
        this.R = R;
        this.J = J;
        this.omegamax = omegamax;
        this.Tmax = Tmax;
        this.rX = rX;
        this.rY = rY;
        this.normalize = normalize;

    }

    public double[] powerSetting(final Vector3 desiredAccel, final Vector3 desiredVel, final double[] correction, final double alpha) {

        /*----------------------------Unpacking values---------------------------------------*/

        final double d2xdt2 = desiredAccel.x;
        final double dxdt = desiredVel.x;

        final double d2ydt2 = desiredAccel.y;
        final double dydt = desiredVel.y;

        final double d2alphadt2 = desiredAccel.theta;
        final double dalphadt = desiredVel.theta;
//        final double alpha = desiredPos.theta;

        /*----------------------------P_x and P_y calculations--------------------------------*/

        final double m_R2_omegamax = m*Math.pow(R, 2)*omegamax;
        final double R_Tmax_omegamax = 4*R*Tmax*omegamax;

        final double P_x_uncorrected = (-m_R2_omegamax*d2xdt2 - 4*Tmax*dxdt)/(R_Tmax_omegamax);
        final double P_y_uncorrected = (-m_R2_omegamax*d2ydt2 - 4*Tmax*dydt)/(R_Tmax_omegamax);
//        final double P_y = -0.849257;
//        System.out.println("R_Tmax_omegamax: " + R_Tmax_omegamax); //9.891 - 9.891
//        System.out.println("4*Tmax*dydt: " + 4*Tmax*dydt);
//        System.out.println("Tmax: " + Tmax);
//        System.out.println("dydt: " + dydt);

        /*----------------------------P_alpha calculations-----------------------------------*/

        final double J_R2_omegamax = J*Math.pow(R, 2)*omegamax;
        final double Tmax_rX2 = Tmax*Math.pow(rX, 2);
        final double Tmax_rX_rY = Tmax*rX*rY;
        final double Tmax_rY2 = Tmax*Math.pow(rY,2);

        final double P_alpha_uncorrected = (-J_R2_omegamax*d2alphadt2 - 4*Tmax_rX2*dalphadt - 8*Tmax_rX_rY*dalphadt - 4*Tmax_rY2*dalphadt)/(R_Tmax_omegamax*(rX+rY));

//        System.out.println("Power profile: " + Arrays.toString(new double[]{P_x_uncorrected, P_y_uncorrected, P_alpha_uncorrected}));

        final double P_x = P_x_uncorrected
                + correction[0]
                ;
        final double P_y = P_y_uncorrected
                + correction[1]
                ;
        final double P_alpha = P_alpha_uncorrected
                + correction[2]
                ;

//        System.out.println("Final power setting: " + Arrays.toString(new double[]{P_x, P_y, P_alpha}));


//        /*----------------------------P_x P_y -> P_X P_Y-------------------------------------*/

//        System.out.println("Actual alpha: " + alpha);
//        final double P_X = P_x*Math.cos(alpha) + P_y*Math.sin(alpha);
//        final double P_Y = -P_x*Math.sin(alpha) + P_y*Math.cos(alpha);
//
//        System.out.println("P_x P_y: " + Arrays.toString(new double[]{P_x, P_y}));
//        System.out.println("P_X P_Y: " + Arrays.toString(new double[]{P_X, P_Y}));
//        System.out.println("P_alpha: " + P_alpha);
////
////        /*----------------------------P_X P_Y -> P_i ----------------------------------------*/
////
//        final double P_1 = P_Y - P_X + P_alpha;
//        final double P_2 = P_Y + P_X - P_alpha;
//        final double P_3 = P_Y - P_X - P_alpha;
//        final double P_4 = P_Y + P_X + P_alpha;
//
//        System.out.println(Arrays.toString(new double[] {P_1, P_2, P_3, P_4}));
//
//        return new double[] {P_1, P_2, P_3, P_4};

        return new double[]{P_x, P_y, P_alpha};
    }
}

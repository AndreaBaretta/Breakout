package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.util.Config;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.quixilver8404.simulator.CoordinateTransformations;
import org.quixilver8404.breakout.util.Vector3;

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

    public static double[] toRawPowerSettings(final Vector3 desiredAccel, final Vector3 desiredVel, final double alpha) {
        final double d2xdt2 = desiredAccel.x;
        final double dxdt = desiredVel.x;

        final double d2ydt2 = desiredAccel.y;
        final double dydt = desiredVel.y;

        final double d2alphadt2 = desiredAccel.theta;
        final double dalphadt = desiredVel.theta;

        final double m_R2_omegamax = Config.MASS *Math.pow(Config.WHEEL_RADIUS, 2)*Config.OMEGA_MAX;
        final double R_Tmax_omegamax = 4*Config.WHEEL_RADIUS*Config.T_MAX*Config.OMEGA_MAX;

        final double P_x_uncorrected = (-m_R2_omegamax*d2xdt2 - 4*Config.T_MAX*dxdt)/(R_Tmax_omegamax);
        final double P_y_uncorrected = (-m_R2_omegamax*d2ydt2 - 4*Config.T_MAX*dydt)/(R_Tmax_omegamax);

        final double J_R2_omegamax = Config.J*Math.pow(Config.WHEEL_RADIUS, 2)*Config.OMEGA_MAX;
        final double Tmax_rX2 = Config.T_MAX*Math.pow(Config.r_X, 2);
        final double Tmax_rX_rY = Config.T_MAX*Config.r_X*Config.r_Y;
        final double Tmax_rY2 = Config.T_MAX*Math.pow(Config.r_Y,2);

        final double P_alpha_uncorrected = (-J_R2_omegamax*d2alphadt2 - 4*Tmax_rX2*dalphadt - 8*Tmax_rX_rY*dalphadt - 4*Tmax_rY2*dalphadt)/(R_Tmax_omegamax*(Config.r_X+Config.r_Y));

        final double P_x = P_x_uncorrected;
        final double P_y = P_y_uncorrected;
        final double P_alpha = P_alpha_uncorrected;

        final double[] P_rel = CoordinateTransformations.toRelativeCoordinates(new ArrayRealVector(new double[]{P_x, P_y}), alpha).toArray();
        final double P_X = P_rel[0];
        final double P_Y = P_rel[1];

        final double P_1_ = P_Y - P_X + P_alpha;
        final double P_2_ = P_Y + P_X - P_alpha;
        final double P_3_ = P_Y - P_X - P_alpha;
        final double P_4_ = P_Y + P_X + P_alpha;

        return new double[] {P_1_, P_2_, P_3_, P_4_};
    }

    public double[] powerSetting(final Vector3 desiredAccel, final Vector3 desiredVel, final double[] correction, final double alpha) {
        final double d2xdt2 = desiredAccel.x;
        final double dxdt = desiredVel.x;

        final double d2ydt2 = desiredAccel.y;
        final double dydt = desiredVel.y;

        final double d2alphadt2 = desiredAccel.theta;
        final double dalphadt = desiredVel.theta;

        final double m_R2_omegamax = m*Math.pow(R, 2)*omegamax;
        final double R_Tmax_omegamax = 4*R*Tmax*omegamax;

        final double P_x_uncorrected = (-m_R2_omegamax*d2xdt2 - 4*Tmax*dxdt)/(R_Tmax_omegamax);
        final double P_y_uncorrected = (-m_R2_omegamax*d2ydt2 - 4*Tmax*dydt)/(R_Tmax_omegamax);

        final double J_R2_omegamax = J*Math.pow(R, 2)*omegamax;
        final double Tmax_rX2 = Tmax*Math.pow(rX, 2);
        final double Tmax_rX_rY = Tmax*rX*rY;
        final double Tmax_rY2 = Tmax*Math.pow(rY,2);

        final double P_alpha_uncorrected = (-J_R2_omegamax*d2alphadt2 - 4*Tmax_rX2*dalphadt - 8*Tmax_rX_rY*dalphadt - 4*Tmax_rY2*dalphadt)/(R_Tmax_omegamax*(rX+rY));

        final double P_x = P_x_uncorrected + correction[0];
        final double P_y = P_y_uncorrected + correction[1];
        final double P_alpha = P_alpha_uncorrected + correction[2];

//        System.out.println("P_x: " + P_x);
//        System.out.println("P_y: " + P_y);
//        System.out.println("P_alpha: " + P_alpha);

        final double[] P_rel = CoordinateTransformations.toRelativeCoordinates(new ArrayRealVector(new double[]{P_x, P_y}), alpha).toArray();
        final double P_X = P_rel[0];
        final double P_Y = P_rel[1];

//        System.out.println("alpha: " + alpha);
//        System.out.println("P_X: " + P_X);
//        System.out.println("P_Y: " + P_Y);
//        System.out.println("P_alpha: " + P_alpha);

        final double P_1_ = P_Y - P_X + P_alpha;
        final double P_2_ = P_Y + P_X - P_alpha;
        final double P_3_ = P_Y - P_X - P_alpha;
        final double P_4_ = P_Y + P_X + P_alpha;

        final double P_1, P_2, P_3, P_4;

        final double[] P_array = new double[]{P_1_, P_2_, P_3_, P_4_};
        if (Math.abs(P_1_) > 1 || Math.abs(P_2_) > 1 || Math.abs(P_3_) > 1 || Math.abs(P_4_) > 1) {
            double max_P = 0;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(P_array[i]) > max_P) {
                    max_P = Math.abs(P_array[i]);
                }
            }
            P_1 = P_1_/max_P;
            P_2 = P_2_/max_P;
            P_3 = P_3_/max_P;
            P_4 = P_4_/max_P;
        } else {
            P_1 = P_1_;
            P_2 = P_2_;
            P_3 = P_3_;
            P_4 = P_4_;
        }

        return new double[]{P_1, P_2, P_3, P_4};
    }
}

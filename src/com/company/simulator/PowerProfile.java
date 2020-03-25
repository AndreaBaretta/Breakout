package com.company.simulator;

import com.company.simulator.CoordinateTransformations;
import com.company.simulator.Vector3;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class PowerProfile {

    final double m;
    final double R;
    final double J;
    final double omegamax;
    final double Tmax;
    final boolean normalize;
    final double rX;
    final double rY;

    public PowerProfile(final double m, final double R, final double J, final double omegamax,
                        final double Tmax, final boolean normalize, final double rX, final double rY) {
        this.m = m;
        this.R = R;
        this.J = J;
        this.omegamax = omegamax;
        this.Tmax = Tmax;
        this.normalize = normalize;
        this.rX = rX;
        this.rY = rY;

    }

    public double[] powerSetting(final Vector3 desiredAccel, final Vector3 desiredVel, final Vector3 desiredPos) {

        /*----------------------------P_x and P_y calculations--------------------------------*/

        final double d2xdt2 = desiredAccel.x;
        final double dxdt = desiredVel.x;
        final double x = desiredPos.x;

        final double d2ydt2 = desiredAccel.y;
        final double dydt = desiredVel.y;
        final double y = desiredPos.y;

        final double d2alphadt2 = desiredAccel.theta;
        final double dalphadt = desiredVel.theta;
        final double alpha = desiredPos.theta;

        final double m_R2_omegamax = m*Math.pow(R, 2)*omegamax;
        final double R_Tmax_omegamax = 4*R*Tmax*omegamax;

        final double P_x = (-m_R2_omegamax*d2xdt2 - 4*Tmax*dxdt + 4*Tmax*y*dalphadt)/(R_Tmax_omegamax);
        final double P_y = (-m_R2_omegamax*d2ydt2 - 4*Tmax*dydt + 4*Tmax*x*dalphadt)/(R_Tmax_omegamax);

        /*----------------------------P_alpha calculations-----------------------------------*/

        final double J_R2_omegamax = J*Math.pow(R, 2)*omegamax;
        final double Tmax_rX2 = Tmax*Math.pow(rX, 2);
        final double Tmax_rX_rY = Tmax*rX*rY;
        final double Tmax_rY2 = Tmax*Math.pow(rY,2);

        final double P_alpha = (-J_R2_omegamax*d2alphadt2 - 4*Tmax_rX2*dalphadt - 8*Tmax_rX_rY*dalphadt - 4*Tmax_rY2*dalphadt)/(R_Tmax_omegamax*(rX+rY));

        /*----------------------------P_x P_y -> P_X P_Y-------------------------------------*/

        final RealVector P_xP_y = new ArrayRealVector(new double[] {P_x,P_y});
        final RealVector P_XP_Y = CoordinateTransformations.toRelativeCoordinates(P_xP_y, alpha);

        final double[] P_XP_Y_array = P_XP_Y.toArray();
        final double P_X = P_XP_Y_array[0];
        final double P_Y = P_XP_Y_array[1];

        /*----------------------------P_X P_Y -> P_i ----------------------------------------*/

        final double P_1 = P_Y - P_X + P_alpha;
        final double P_2 = P_Y + P_X - P_alpha;
        final double P_3 = P_Y - P_X - P_alpha;
        final double P_4 = P_Y + P_X + P_alpha;

        return new double[] {P_1, P_2, P_3, P_4};
    }
}

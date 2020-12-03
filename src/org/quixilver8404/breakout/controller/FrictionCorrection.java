package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class FrictionCorrection {

    public static double[] correction(final Vector3 vel, final Vector3 deltaPos, final double alpha, final double[] powerSettings) {
        final double[] vel_w = new double[]{
                getWheelVel(vel, Config.r_X, Config.r_Y, -Math.PI / 4, Config.WHEEL_RADIUS, alpha),
                getWheelVel(vel, -Config.r_X, Config.r_Y, Math.PI / 4, Config.WHEEL_RADIUS, alpha),
                getWheelVel(vel, -Config.r_X, -Config.r_Y, -Math.PI / 4, Config.WHEEL_RADIUS, alpha),
                getWheelVel(vel, Config.r_X, -Config.r_Y, Math.PI / 4, Config.WHEEL_RADIUS, alpha)
        };

        final double[] correction = new double[4];
        final double[] finalPowerSettings = new double[4];
        for (int i = 0; i < 4; i++) {
            final double dynamicCorrection = Math.signum(vel_w[i])*Config.P_dynamic;

            if (Math.abs(deltaPos.x) <= 1e-3 && Math.abs(deltaPos.y) <= 1e-3 && Math.abs(deltaPos.theta) <= 1e-3) {
                System.out.println("Error at 0");
                correction[i] = 0;
            } else if (Math.abs(vel_w[i]) <= 1e-9 && Math.abs(powerSettings[i] + dynamicCorrection) <= Config.P_static) {
                correction[i] = -powerSettings[i] + Math.signum(powerSettings[i])*Config.P_static;
            } else {
                correction[i] = dynamicCorrection;
            }

            finalPowerSettings[i] = powerSettings[i] + correction[i];
            System.out.println("i: " + i + " power setting: " + powerSettings[i] + " correction: "+ correction[i] + " total: " + finalPowerSettings[i]);
        }


        return finalPowerSettings;
//        return new double[]{0.2,0.2,0.2,0.31};
    }

    public static double getWheelVel(final Vector3 vel, final double rX, final double rY, final double phi, final double r, final double alpha) {
        final double sin_a = Math.sin(alpha);
        final double cos_a = Math.cos(alpha);
        final double vdX = -alpha*rY + (vel.x*cos_a + vel.y*sin_a);
        final double vdY = alpha*rX + (-vel.x*sin_a + vel.y*cos_a);
        final double vw = -vdY - vdX*Math.tan(phi);
        return vw/r;
    }
}

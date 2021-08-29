package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class FrictionCorrection {

    public static double[] correction(final Vector3 vel, final Vector3 deltaPos, final double alpha, final double[] powerSettings, final boolean stopAtZero) {
//        System.out.println("rX: " + Config.r_X + " rY: " + Config.r_Y);
        final double[] vel_w = new double[]{
                getWheelVel(vel, Config.r_X, Config.r_Y, -Math.PI / 4, Config.WHEEL_RADIUS, alpha),
                getWheelVel(vel, -Config.r_X, Config.r_Y, Math.PI / 4, Config.WHEEL_RADIUS, alpha),
                getWheelVel(vel, -Config.r_X, -Config.r_Y, -Math.PI / 4, Config.WHEEL_RADIUS, alpha),
                getWheelVel(vel, Config.r_X, -Config.r_Y, Math.PI / 4, Config.WHEEL_RADIUS, alpha)
        };

        final double[] finalPowerSettings = new double[4];

//        System.out.println("Error: " + deltaPos.toString());

        for (int i = 0; i < 4; i++) {
            final double dynamicCorrection = Math.signum(vel_w[i])*Config.P_DYNAMIC;

            if (Math.abs(deltaPos.x) <= 0.07 && Math.abs(deltaPos.y) <= 0.07 && Math.abs(deltaPos.theta)*180/Math.PI <= 2 && stopAtZero) {
                finalPowerSettings[i] = 0;
            } else if (Math.abs(vel_w[i]) <= 1e-4 && Math.abs(powerSettings[i] + dynamicCorrection) <= Config.P_STATIC) {//TODO FIGURE THIS OUT
//                System.out.println("Can't overcome static");
                finalPowerSettings[i] = Math.signum(powerSettings[i])*Config.P_STATIC/Config.VOLTAGE_SCALE;
            } else {
//                System.out.println("Dynamic");
                finalPowerSettings[i] = (dynamicCorrection + powerSettings[i])/Config.VOLTAGE_SCALE;
            }
        }
        finalPowerSettings[1] *= Config.FRICTION_SCALAR_FACTOR_WHEEL_1;
        finalPowerSettings[2] *= Config.FRICTION_SCALAR_FACTOR_WHEEL_2;
        finalPowerSettings[3] *= Config.FRICTION_SCALAR_FACTOR_WHEEL_3;
        finalPowerSettings[4] *= Config.FRICTION_SCALAR_FACTOR_WHEEL_4;
        return finalPowerSettings;
    }

    public static double getWheelVel(final Vector3 vel, final double rX, final double rY, final double phi, final double r, final double alpha) {
        final double sin_a = Math.sin(alpha);
        final double cos_a = Math.cos(alpha);
        final double vdX = -vel.theta*rY + (vel.x*cos_a + vel.y*sin_a);
        final double vdY = vel.theta*rX + (-vel.x*sin_a + vel.y*cos_a);
        final double vw = -vdY - vdX*Math.tan(phi);
        return vw/r;
    }
}

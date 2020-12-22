package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class FrictionCorrection {

    public static double[] correction(final Vector3 vel, final Vector3 deltaPos, final double alpha, final double[] powerSettings) {
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

            if (Math.abs(deltaPos.x) <= 1e-3 && Math.abs(deltaPos.y) <= 1e-3 && Math.abs(deltaPos.theta) <= 1e-3) {
//                System.out.println("Error at 0");
//                System.exit(0);
                finalPowerSettings[i] = powerSettings[i];
            } else if (Math.abs(vel_w[i]) <= 1e-4 && Math.abs(powerSettings[i] + dynamicCorrection) <= Config.P_STATIC) {
//                System.out.println("Can't overcome static");
                finalPowerSettings[i] = Math.signum(powerSettings[i])*Config.P_STATIC;
            } else {
//                System.out.println("Dynamic");
                finalPowerSettings[i] = dynamicCorrection + powerSettings[i];
            }
//            System.out.println("Math.abs(vel_w[i]) <= 1e-4: " + (Math.abs(vel_w[i]) <= 1e-4));
//            System.out.println("Math.abs(powerSettings[i] + dynamicCorrection) <= Config.P_STATIC: " + (Math.abs(powerSettings[i] + dynamicCorrection) <= Config.P_STATIC));
//            System.out.println("i: " + i + " power setting: " + powerSettings[i] + " correction: " + (finalPowerSettings[i] - powerSettings[i]) + " total: " + finalPowerSettings[i] + " wheel velocity: " + vel_w[i] + " powerSetting + dynamicCorrection: " + (powerSettings[i] + dynamicCorrection));

        }
//        System.out.println("Robot velocity: " + vel.toString());

        if (deltaPos.theta > 0) {
            System.exit(0);
        }


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

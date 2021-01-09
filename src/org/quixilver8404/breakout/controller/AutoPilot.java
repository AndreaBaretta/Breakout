package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.feedforward.*;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.TreeSet;

public class AutoPilot {

    public final Controller controller;
    public final PowerProfile powerProfile;
    protected double prev_s = 0;
    protected boolean reachedEnd = false;
    protected LinearSegment segment;
    protected Vector3 desiredPos;

    public AutoPilot(final Config robotConfig, final boolean v1) {
        robotConfig.set();
        if (v1) {
            controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        } else {
            controller = new Controller(Controller.computeK2(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        }
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, false);
    }

    public void setDesiredPos(final Vector3 desiredPos/*, final Vector3 curPos*/) {
        prev_s = 0;
        reachedEnd = false;
        this.desiredPos = desiredPos;
    }


    public double[] correction(final Vector3 pos, final Vector3 vel) {

//        System.out.println("Robot Position: " + pos.toString());

        final double[] correction = controller.correction(Vector3.subtractVector2(pos, desiredPos),
                Vector3.subtractVector(vel, new Vector3(0,0,0)));

        final double[] powerSettings = powerProfile.powerSetting(new Vector3(0,0,0), new Vector3(0,0,0), correction, pos.theta);

        final double[] frictionAdjustedPowerSettings = FrictionCorrection.correction(vel, Vector3.subtractVector2(pos, desiredPos), pos.theta, powerSettings, true);

//        System.out.println("Friction adjusted power: " + Arrays.toString(frictionAdjustedPowerSettings));
//        System.out.println(frictionAdjustedPowerSettings);
        return frictionAdjustedPowerSettings;
//        return powerSettings;
//        return new double[]{-1,-1,-1,-1};
    }

}

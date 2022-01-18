package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.feedforward.*;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.TreeSet;

public class AutoPilot {

    public Controller controller;
    public final PowerProfile powerProfile;
    public final Config config;
    protected double prev_s = 0;
    protected boolean reachedEnd = false;
    protected Vector3 desiredPos;

    public AutoPilot(final Config robotConfig, final boolean v1) {
        config = robotConfig;
        robotConfig.set();
        if (v1) {
            controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        } else {
            controller = new Controller(Controller.computeK2(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        }
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, false);
    }

    public void setDesiredPos(final Vector3 desiredPos) {
        prev_s = 0;
        reachedEnd = false;
        this.desiredPos = toBreakoutCoords(desiredPos);
    }

    public double[] last_correction_power = new double[]{0,0,0};
    public double[] last_motor_power = new double[]{0,0,0,0};
    public double[] last_frc_adj_motor_power = new double[]{0,0,0,0};

    public double[] correction(final Vector3 pos, final Vector3 vel) {

        System.out.println("Pos: " + pos.toString());
        System.out.println("Pos error: " + Vector3.subtractVector2(pos, desiredPos));
        System.out.println("Vel: " + vel.toString());
        System.out.println("Vel error: " + Vector3.subtractVector(vel, new Vector3(0,0,0)));

        final double[] correction = controller.correction(Vector3.subtractVector2(pos, desiredPos),
                Vector3.subtractVector(vel, new Vector3(0,0,0)));

        last_correction_power = correction;

        System.out.println("Feedback correction: p_alphaa=" + correction[2]);

        final double[] powerSettings = powerProfile.powerSetting(new Vector3(0,0,0), new Vector3(0,0,0), correction, pos.theta);

        System.out.println("Motor power before frictiom adjustment: " + Arrays.toString(powerSettings));

        last_motor_power = powerSettings;

        final double[] frictionAdjustedPowerSettings = FrictionCorrection.correction(vel, Vector3.subtractVector2(pos, desiredPos), pos.theta, powerSettings, true);

        last_frc_adj_motor_power = frictionAdjustedPowerSettings;
//        System.out.println("Friction adjusted power: " + Arrays.toString(frictionAdjustedPowerSettings));
//        System.out.println(frictionAdjustedPowerSettings);
        return frictionAdjustedPowerSettings;
//        return powerSettings;
//        return new double[]{-1,-1,-1,-1};
    }

    public void setVoltage(final double voltage) {
        config.setVoltage(voltage);
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
    }

    public static Vector3 toBreakoutCoords(final Vector3 pos) {
        return new Vector3(pos.x, pos.y, pos.theta - Math.PI/2);
    }

}

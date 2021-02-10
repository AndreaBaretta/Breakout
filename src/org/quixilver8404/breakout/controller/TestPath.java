package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.feedforward.RobotState;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class TestPath {

    final double v;
    protected final Config config;
    public Controller controller;
    public final PowerProfile powerProfile;

    public TestPath(final double vel, final Config config) {
        v = vel;
        this.config = config;
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, true);
    }

    protected double t = 0;

    public Vector3 last_pos = new Vector3(1,0,0);

    public double[] last_correction_power = new double[]{0,0,0};
    public double[] last_motor_power = new double[]{0,0,0,0};
    public double[] last_frc_adj_motor_power = new double[]{0,0,0,0};

    public double[] iterate(final Vector3 pos, final Vector3 vel, final double dt) {

        final Vector3 des_pos = new Vector3(0.8*Math.cos(v*t), 0.8*Math.sin(v*t), -Math.PI/2);
        final Vector3 des_vel = new Vector3(-0.8*Math.sin(v*t)*v, 0.8*Math.cos(v*t)*v, 0);
        final Vector3 des_acc = new Vector3(-0.8*Math.cos(v*t)*v*v, -0.8*Math.sin(v*t)*v*v, 0);

        last_pos = des_pos;

        final double[] correction = controller.correction(Vector3.subtractVector2(pos, des_pos),
                Vector3.subtractVector(vel, des_vel));

        last_correction_power = correction;

        final double[] powerSettings = powerProfile.powerSetting(des_acc, des_vel, correction, pos.theta);

        last_motor_power = powerSettings;

        final double[] frictionAdjustedPowerSettings;
        frictionAdjustedPowerSettings = FrictionCorrection.correction(vel, Vector3.subtractVector2(pos, des_pos), pos.theta, powerSettings, true);

        last_frc_adj_motor_power = frictionAdjustedPowerSettings;

        t += dt;

        return frictionAdjustedPowerSettings;
    }

    public void setVoltage(final double voltage) {
        config.setVoltage(voltage);
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
    }
}
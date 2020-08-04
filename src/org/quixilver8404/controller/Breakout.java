package org.quixilver8404.controller;

import org.quixilver8404.feedforward.Path;
import org.quixilver8404.feedforward.RobotState;
import org.quixilver8404.util.Config;
import org.quixilver8404.util.Vector3;

import java.io.File;

public class Breakout {
    public final Controller controller;
    public final PowerProfile powerProfile;
    public final Path path;

    public Breakout(final File foxtrotFile, final int config) {
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, true);
        path = Path.parseFoxtrot(foxtrotFile, config);
    }

    protected double prev_s = 0;

    public double[] iterate(final Vector3 pos, final Vector3 vel, final double dt) {
        final double s = path.calcS(pos.x, pos.y);
        final double s_dot = (s - prev_s)/dt;
        final double s_dot_dot = path.calcAccelerationCorrection(s, s_dot);

        prev_s = s;

        final RobotState state = path.evaluate(s, s_dot, s_dot_dot);

        final double[] correction = controller.correction(Vector3.subtractVector2(pos, state.pos),
                Vector3.subtractVector(vel, state.vel));

        final double[] powerSettings = powerProfile.powerSetting(state.acc, state.vel, correction, pos.theta);

        return powerSettings;
    }
}

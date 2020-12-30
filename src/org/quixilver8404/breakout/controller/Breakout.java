package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.feedforward.ActionEventListener;
import org.quixilver8404.breakout.feedforward.Path;
import org.quixilver8404.breakout.feedforward.RobotState;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.File;
import java.util.List;

public class Breakout {
    public final Controller controller;
    public final PowerProfile powerProfile;
    public final Path path;
    protected Vector3 lastKnownPos;

    public Breakout(final File foxtrotFile, final int foxtrotConfig, final List<ActionEventListener> actionEventListeners, final Config robotConfig) {
        robotConfig.set();
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, true);
        path = new Path(foxtrotFile, foxtrotConfig, actionEventListeners);
        lastKnownPos = new Vector3(0,0,0);
    }

    protected double prev_s = 0;

    public double[] iterate(final Vector3 pos, final Vector3 vel, final double dt) {
        final double s = path.calcS(pos.x, pos.y);
        final double s_dot = (s - prev_s)/dt;
        final double s_dot_dot = path.calcAccelerationCorrection(s, s_dot);

        prev_s = s;

        final RobotState state = path.evaluate(s, s_dot, s_dot_dot);
        lastKnownPos = state.pos;

        final double[] correction = controller.correction(Vector3.subtractVector2(pos, state.pos),
                Vector3.subtractVector(vel, state.vel));

        final double[] powerSettings = powerProfile.powerSetting(state.acc, state.vel, correction, pos.theta);

        return powerSettings;
    }

    public boolean isFinished() {
        return path.isFinished();
    }

    public Vector3 getLastKnownPos() {
        return lastKnownPos;
    }
}

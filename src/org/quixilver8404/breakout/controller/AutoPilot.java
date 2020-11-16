package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class AutoPilot {

    public final Controller controller;
    public final PowerProfile powerProfile;
    protected Vector3 desiredPos = new Vector3(0,0,0);
    protected final Vector3 desiredVel = new Vector3(0,0,0);
    protected final Vector3 desiredAcc = new Vector3(0,0,0);
    public AutoPilot(final Config robotConfig) {
        robotConfig.set();
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, false);
    }

    public void setDesiredPos(final Vector3 desiredPos) {
        this.desiredPos = desiredPos;
    }

    public double[] correction(final Vector3 pos, final Vector3 vel) {
        final double[] correction = controller.correction(Vector3.subtractVector2(pos, desiredPos),
                Vector3.subtractVector(vel, desiredVel));

        final double[] powerSettings = powerProfile.powerSetting(desiredAcc, desiredVel, correction, pos.theta);

        return powerSettings;
    }
}

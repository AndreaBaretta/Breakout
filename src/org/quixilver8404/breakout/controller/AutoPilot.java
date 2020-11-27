package org.quixilver8404.breakout.controller;

import org.quixilver8404.breakout.feedforward.*;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.util.ArrayList;
import java.util.TreeSet;

public class AutoPilot {

    public final Controller controller;
    public final PowerProfile powerProfile;
    protected double prev_s = 0;
    protected boolean reachedEnd = false;
    protected LinearSegment segment;
    protected Vector3 desiredPos;

    public AutoPilot(final Config robotConfig) {
        robotConfig.set();
        controller = new Controller(Controller.computeK(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y));
        powerProfile = new PowerProfile(Config.MASS, Config.WHEEL_RADIUS, Config.J, Config.OMEGA_MAX, Config.T_MAX, Config.r_X, Config.r_Y, false);
    }

    public void setDesiredPos(final Vector3 desiredPos, final Vector3 curPos) {
        segment = new LinearSegment(
                new ConnectionPoint(curPos.x, curPos.y, AnchorPoint.Heading.CUSTOM, curPos.theta, 1, new ArrayList<ActionEventListener>(), new TreeSet<Integer>()),
                new ConnectionPoint(desiredPos.x, desiredPos.y, AnchorPoint.Heading.CUSTOM, desiredPos.theta, 0, new ArrayList<ActionEventListener>(), new TreeSet<Integer>()),
                0, 0);
        prev_s = 0;
        reachedEnd = false;
        this.desiredPos = desiredPos;
    }

    public double[] correction(final Vector3 pos, final Vector3 vel, final Vector3 acc, final double dt) {

        if (segment.zeroSegment) { return correction2(pos, vel, dt); }

        final double s = segment.calcS(pos);
        final double s_dot = (s - prev_s)/dt;
        final double s_dot_dot = calcAccelerationCorrection(s, s_dot);

        prev_s = s;

        if (s >= segment.getTotalS()) {
            reachedEnd = true;
        }

        if (!reachedEnd) {

            final Vector3 xyPos = segment.getPosition(s);

            final double alpha0 = Vector3.normalizeAlpha(segment.firstPoint.heading);
            final double alpha1 = Vector3.normalizeAlpha(desiredPos.theta);
            final double alpha0_;
            if (alpha1 > alpha0) {
                if (alpha1 - alpha0 >= Math.PI) {
                    alpha0_ = alpha0 + 2*Math.PI;
                } else {
                    alpha0_ = alpha0;
                }
            } else {
                if (alpha0 - alpha1 >= Math.PI) {
                    alpha0_ = alpha0 - 2*Math.PI;
                } else {
                    alpha0_ = alpha0;
                }
            }

            final double alpha = alpha0 + (alpha1 - alpha0_)*(s - segment.s0)/(segment.getTotalS());

            final RobotState state = new RobotState(new Vector3(xyPos.x, xyPos.y, alpha), segment.getVelocity(s, s_dot), segment.getAcceleration(s, s_dot, s_dot_dot));

            final double[] correction = controller.correction(Vector3.subtractVector2(pos, state.pos),
                    Vector3.subtractVector(vel, state.vel), Vector3.subtractVector(acc, state.acc), dt);

            final double[] powerSettings = powerProfile.powerSetting(state.acc, state.vel, correction, pos.theta);

            return powerSettings;
        } else {
            final double[] correction = controller.correction(Vector3.subtractVector2(pos, desiredPos),
                    Vector3.subtractVector(vel, new Vector3(0,0,0)), Vector3.subtractVector(acc, new Vector3(0,0,0)), dt);

            final double[] powerSettings = powerProfile.powerSetting(new Vector3(0,0,0), new Vector3(0,0,0), correction, pos.theta);

            return powerSettings;
        }
    }

    public double[] correction2(final Vector3 pos, final Vector3 vel, final double dt) {
        final double[] correction = controller.correction(Vector3.subtractVector2(pos, desiredPos),
                Vector3.subtractVector(vel, desiredPos), new Vector3(0,0,0)/*REMOVE THIS*/, dt);

        final double[] powerSettings = powerProfile.powerSetting(new Vector3(0,0,0), new Vector3(0,0,0), correction, pos.theta);

        return powerSettings;
    }

    public double calcAccelerationCorrection(final double s, final double s_dot) {
        final double d_s = segment.getTotalS() - s;
        final double v_f = segment.lastPoint.getConfigVelocity();
        final double accToVel = (1/d_s)*(0.5 * Math.pow(v_f - s_dot, 2) + s_dot * (v_f - s_dot));

        if (accToVel <= Config.MAX_SAFE_ACCELERATION*Config.MAX_DECELERATION) {
            return accToVel;
        }

        final double maxAcc = findMaxPossibleAcc(s, s_dot);
        final double acc = Math.tan(Config.ACCELERATION_CORRECTION)*(segment.firstPoint.getConfigVelocity() - s_dot);
        if (acc > maxAcc) {
            return maxAcc;
        } else if (acc < Config.MAX_DECELERATION) {
            return Config.MAX_DECELERATION;
        } else {
            return acc;
        }
    }

    public double findMaxPossibleAcc(final double s, final double s_dot) {
        double s_dot_dot = Config.MAX_ACCELERATION;
        while (true) {
            final RobotState state = new RobotState(segment.getPosition(s), segment.getVelocity(s, s_dot), segment.getAcceleration(s, s_dot, s_dot_dot));
            final Vector3 vel = state.vel;
            final Vector3 acc = state.acc;
            final double[] powerSettings = PowerProfile.toRawPowerSettings(acc, vel, state.pos.theta);
            boolean optimized = true;
            for (double p : powerSettings) {
                if (Math.abs(p) >= 1) {
                    optimized = false;
                }
            }
            if (optimized) {
                return s_dot_dot;
            }
            if (s_dot_dot < Config.MAX_DECELERATION) {
                return 0;
            }
            s_dot_dot -= Config.ACCELERATION_CORRECTION_STEP;
        }
    }
}

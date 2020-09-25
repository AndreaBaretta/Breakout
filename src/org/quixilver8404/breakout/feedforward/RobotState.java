package org.quixilver8404.breakout.feedforward;

import org.quixilver8404.breakout.util.Vector3;

public class RobotState {
    public final Vector3 pos;
    public final Vector3 vel;
    public final Vector3 acc;

    RobotState(final Vector3 pos, final Vector3 vel, final Vector3 acc) {
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
    }
}

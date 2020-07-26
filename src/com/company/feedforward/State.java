package com.company.feedforward;

import com.company.simulator.Vector3;

public class State {
    final Vector3 pos;
    final Vector3 vel;
    final Vector3 acc;

    State(final Vector3 pos, final Vector3 vel, final Vector3 acc) {
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
    }
}

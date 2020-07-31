package com.company.feedforward;

import com.company.simulator.Vector3;

public abstract class Segment {
//    public final ConnectionPoint firstPoint;
//    public final ConnectionPoint lastPoint;
    public final double s0;
    public final double minVelocity;

    Segment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0, final double configVelocity) {
//        this.firstPoint = firstPoint;
//        this.lastPoint = lastPoint;
//        firstPoint.setNextSegment(this);
//        lastPoint.setPrevSegment(this);
        this.s0 = s0;
        minVelocity = Math.min(configVelocity, calcMinVelocity());
    }

    public abstract Vector3 getPosition(final double s);

    public abstract Vector3 getVelocity(final double s, final double s_dot);

    public abstract Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot);

    public abstract double getTotalS();

    public double getEndS() {
        return s0 + getTotalS();
    };

    public boolean inRange(final double s) {
        if (s0 <= s && s <= getEndS()) {
            return true;
        } else {
            return false;
        }
    }

    public abstract double calcMinVelocity();

    public abstract double calcS(final double x, final double y);

    public double calcS(final Vector3 pos) {
        return calcS(pos.x, pos.y);
    }
}

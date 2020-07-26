package com.company.feedforward;

import com.company.simulator.Vector3;

public abstract class Segment {
    final ConnectionPoint firstPoint;
    final ConnectionPoint lastPoint;
    final double s0;

    Segment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0) {
        this.firstPoint = firstPoint;
        this.lastPoint = lastPoint;
        firstPoint.setNextSegment(this);
        lastPoint.setPrevSegment(this);
        this.s0 = s0;
    }

    public abstract Vector3 getPosition(final double s);

    public abstract Vector3 getVelocity(final double s, final double s_dot);

    public abstract Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot);

//    public abstract double getAlpha(final double s);
//
//    public abstract double getAngularVelocity(final double s);
//
//    public abstract double getAngularAcceleration(final double s);

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
    };
}

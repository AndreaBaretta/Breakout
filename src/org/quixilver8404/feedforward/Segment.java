package org.quixilver8404.feedforward;

import org.quixilver8404.util.Vector3;

public abstract class Segment {
    public final ConnectionPoint firstPoint;
    public final ConnectionPoint lastPoint;
    public final double s0;
    public final double minVelocity;
    public final boolean zeroSegment;

    Segment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0, final double configVelocity) {
        this.firstPoint = firstPoint;
        this.lastPoint = lastPoint;
        this.s0 = s0;
        minVelocity = Math.min(configVelocity, calcMinVelocity());
        if (Math.abs(firstPoint.x - lastPoint.x) < 1e-12 && Math.abs(firstPoint.y - lastPoint.y) < 1e-12) {
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
    }

    public abstract Vector3 getPosition(final double s);

    public abstract Vector3 getVelocity(final double s, final double s_dot);

    public abstract Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot);

    public abstract double getTotalS();

    public double getEndS() {
//        if (zeroSegment) {
//            return s0;
//        }
        return s0 + getTotalS();
    }

    public boolean inRange(final double s) {
        if (zeroSegment) return false;
        if (s0 <= s && s <= getEndS() + 1e-12) {
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

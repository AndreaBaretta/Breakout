package org.quixilver8404.breakout.feedforward;

import org.quixilver8404.breakout.util.Vector3;

public abstract class Segment {
    public final double s0;
    public final boolean zeroSegment;
    protected int index;

    Segment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint,
            final double s0) {
        this.s0 = s0;
        if (Math.abs(firstPoint.x - lastPoint.x) < 1e-12 &&
                Math.abs(firstPoint.y - lastPoint.y) < 1e-12) {
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
        index = 0;
    }

    public abstract Vector3 getPosition(final double s);

    public abstract Vector3 getVelocity(final double s, final double s_dot);

    public abstract Vector3 getAcceleration(final double s, final double s_dot,
                                            final double s_dot_dot);

    public abstract double getTotalS();

    public double getEndS() {
        return s0 + getTotalS();
    }

    public boolean inRange(final double s) {
        if (zeroSegment) return false;
        if (s0 - 1e-12 <= s && s <= getEndS() + 1e-12) {
            return true;
        } else {
            return false;
        }
    }

    public abstract double calcS(final double x, final double y);

    public double calcS(final Vector3 pos) {
        return calcS(pos.x, pos.y);
    }

    public abstract double getMaxVelocity();

    public void configurePoints(final ConnectionPoint firstPoint,
                                final ConnectionPoint lastPoint) {
        firstPoint.setMaxVelocity(getMaxVelocity());
        firstPoint.setS(s0);
        lastPoint.setMaxVelocity(getMaxVelocity());
        lastPoint.setS(getEndS());
    }

    public void setIndex(final int index) {
        this.index = index;
    }
}

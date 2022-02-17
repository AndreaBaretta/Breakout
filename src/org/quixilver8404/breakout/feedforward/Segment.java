package org.quixilver8404.breakout.feedforward;

import org.quixilver8404.breakout.util.Vector3;

public abstract class Segment {
    public final ConnectionPoint firstPoint;
    public final ConnectionPoint lastPoint;
    public final double s0;
    public final boolean zeroSegment;
    public final int index;

    Segment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0, final int index) {
        this.firstPoint = firstPoint;
        this.lastPoint = lastPoint;
        this.s0 = s0;
        if (Math.abs(firstPoint.x - lastPoint.x) < 1e-12 && Math.abs(firstPoint.y - lastPoint.y) < 1e-12) { //TODO: Potential bug here for circles that go all the way around
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
        this.index = index;
    }

    public abstract Vector3 getPosition(final double s);

    public abstract Vector3 getVelocity(final double s, final double s_dot);

    public abstract Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot);

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

    public abstract boolean isPointSegment();

    public abstract double getMaxVelocity();

    public void configurePoints() {
        if (Double.isNaN(firstPoint.getMaxVelocity())) {
            firstPoint.setMaxVelocity(getMaxVelocity());
        }
        firstPoint.setS(s0);
        lastPoint.setS(getEndS());
        if (getMaxVelocity() == 0) {
            lastPoint.setMaxVelocity(0);
        }
    }
}

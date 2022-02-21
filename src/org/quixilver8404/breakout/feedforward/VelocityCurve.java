package org.quixilver8404.breakout.feedforward;

public class VelocityCurve {
    public final double s0;
    public final double s1;
    public final boolean zeroSegment;
    public final double v0;
    public final double v1;
    public final double acceleration;
    protected int index;

    VelocityCurve(final double s0, final double s1, final double v0, final double v1) {
        this.s0 = s0;
        this.s1 = s1;
        if (s1 - s0 <= 1e-12) {
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
        this.v0 = v0;
        this.v1 = v1;
        acceleration = (Math.pow(v1, 2) - Math.pow(v0, 2))/(2*(s1-s0));
    }

    public String toString() {
        return "(s0=" + s0 + ", s1=" + s1 + ", v=" + v0 + ", nextV=" + v1 + ", index=" + index + ")";
    }

    public boolean inRange(final double s) {
        if (s0 <= s && s <= s1 + 1e-12) {
            return true;
        } else {
            return false;
        }
    }

    public double getVelocity(final double s) {
        return Math.sqrt(2*acceleration*(Math.max(s,s0)-s0) + Math.pow(v0, 2));
    }

    public double distS(final double s) {
        return s1 - s;
    }

    public void setIndex(final int index) {
        this.index = index;
    }

    public int getIndex() {
        return index;
    }
}

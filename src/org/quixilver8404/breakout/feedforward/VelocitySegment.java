package org.quixilver8404.breakout.feedforward;

public class VelocitySegment {
    public final double s0;
    public final double s1;
    public final boolean zeroSegment;
    public final VelocityPoint p0;
    public final VelocityPoint p1;
    public final int index;
    public final double v0;
    public final double v1;
    public final double acceleration;

    VelocitySegment(final VelocityPoint p0, final VelocityPoint p1, final int index) {
        s0 = p0.getS();
        s1 = p1.getS();
        if (s1 - s0 <= 1e-12) {
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
        this.p0 = p0;
        this.p1 = p1;
        this.index = index;
        v0 = Math.min(p0.getConfigVelocity(), p0.getMaxVelocity());
        v1 = Math.min(p0.getConfigVelocity(), p0.getMaxVelocity());
        acceleration = (Math.pow(v1, 2) - Math.pow(v0, 2))/(2*(s1-s0));
    }

    public String toString() {
        return "(s0=" + s0 + ", s1=" + s1 + ", v=" + Math.min(p0.getMaxVelocity(), p0.getConfigVelocity()) + ", nextV=" + Math.min(p1.getMaxVelocity(), p1.getConfigVelocity()) + ")";
    }

    public boolean inRange(final double s) {
        if (s0 <= s && s <= s1 + 1e-12) {
            return true;
        } else {
            return false;
        }
    }

    public double getVelocity(final double s) {
        return Math.sqrt(2*acceleration*(s-s0) + Math.pow(v0, 2));
    }

    public double distS(final double s) {
        return s1 - s;
    }
}

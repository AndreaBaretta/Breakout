package org.quixilver8404.feedforward;

public class VelocitySegment {
    public final double s0;
    public final double s1;
    public final double velocity;
    public final boolean zeroSegment;
    public final VelocityPoint p0;
    public final VelocityPoint p1;

    VelocitySegment(final double s0, final double s1, final double segmentMinVelocity, final VelocityPoint p0, final VelocityPoint p1) {
        this.s0 = s0;
        this.s1 = s1;
        this.velocity = Math.min(segmentMinVelocity, p0.getConfigVelocity());
        if (s1 - s0 <= 1e-12) {
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
        this.p0 = p0;
        this.p1 = p1;
    }

    public String toString() {
        return "(s0=" + s0 + ", s1=" + s1 + ", v=" + velocity + ", nextV=" + getNextVelocity() + ")";
    }

    public boolean inRange(final double s) {
        if (s0 <= s && s <= s1 + 1e-12) {
            return true;
        } else {
            return false;
        }
    }

    public double getNextVelocity() {
        return Math.min(p1.getMinVelocity(), p1.getConfigVelocity());
    }
}

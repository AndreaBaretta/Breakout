package org.quixilver8404.feedforward;

public class VelocitySegment {
    final double s0;
    final double s1;
    protected double minVelocity;
    final boolean zeroSegment;
    final int index;

    VelocitySegment(final double s0, final double s1, final double minVelocity, final int index) {
        this.s0 = s0;
        this.s1 = s1;
        this.minVelocity = minVelocity;
        if (s1 - s0 <= 1e-12) {
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
        this.index = index;
    }

    public void setMinVelocity(final double minVelocity) {
        this.minVelocity = Math.min(this.minVelocity, minVelocity);
    }

    public String toString() {
        return "(s0=" + s0 + ", s1=" + s1 + ", minV=" + minVelocity + ", i=" + index + ")";
    }

    public boolean inRange(final double s) {
        if (s0 <= s && s <= s1 + 1e-12) {
            return true;
        } else {
            return false;
        }
    }
}

package org.quixilver8404.feedforward;

public class VelocitySegment {
    final double s0;
    final double s1;
    protected double minVelocity;

    VelocitySegment(final double s0, final double s1, final double minVelocity) {
        this.s0 = s0;
        this.s1 = s1;
        this.minVelocity = minVelocity;
    }

    public void setMinVelocity(final double minVelocity) {
        this.minVelocity = Math.min(this.minVelocity, minVelocity);
    }
}

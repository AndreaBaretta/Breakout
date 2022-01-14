package org.quixilver8404.breakout.feedforward;

public class VelocitySegment {
    public final double s0;
    public final double s1;
    public final boolean zeroSegment;
    public final VelocityPoint p0;
    public final VelocityPoint p1;
    public final int index;
    protected boolean startedAcceleration;

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
        startedAcceleration = false;
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

    public boolean hasStartedAcceleration() {
        return startedAcceleration;
    }

    public void startAcceleration() {
        startedAcceleration = true;
    }

    public NextVCurVDistS getNextVelocity(final double s) {
        return new NextVCurVDistS(Math.min(p1.getMaxVelocity(), p1.getConfigVelocity()), Math.min(p0.getMaxVelocity(), p0.getConfigVelocity()), s1 - s);
    }

    public double getNextV() {
        return Math.min(p1.getMaxVelocity(), p1.getConfigVelocity());
    }

    public class NextVCurVDistS {
        final double nextV;
        final double curV;
        final double distS;
        public NextVCurVDistS(final double nextV, final double curV, final double distS) {
            this.curV = curV;
            this.nextV = nextV;
            this.distS = distS;
        }
    }
}

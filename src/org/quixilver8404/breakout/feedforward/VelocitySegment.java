package org.quixilver8404.breakout.feedforward;

import java.util.ArrayList;
import java.util.List;

public class VelocitySegment {

    public final VelocityPoint p0;
    public final VelocityPoint p1;
    public final VelocityPoint constraintPt1;
    public final VelocityPoint constraintPt2;
    public final int index;

    protected double s0;
    protected double s1;
    protected boolean zeroSegment;
    protected double v0;
    protected double v1;
    protected List<VelocityCurve> velocityCurves;

    VelocitySegment(final VelocityPoint p0, final VelocityPoint p1, final VelocityPoint constraintPt1, final VelocityPoint constraintPt2, final int index) {
        assert(constraintPt2.getS() >= constraintPt1.getS());
        this.p0 = p0;
        this.p1 = p1;
        this.constraintPt1 = constraintPt1;
        this.constraintPt2 = constraintPt2;
        this.index = index;
    }

    public void set() {
        s0 = p0.getS();
        s1 = p1.getS();
        if (s1 - s0 <= 1e-12) {
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
        if (p0.getMaxVelocity() == 0) { // 0 velocity point --> Ignore 0 vel, try to achieve user-defined velocity, no matter how stupid
            v0 = p0.getConfigVelocity();
        } else { // Normal scenario
            v0 = Math.min(p0.getConfigVelocity(), p0.getMaxVelocity());
        }
        v1 = Math.min(p1.getConfigVelocity(), p1.getMaxVelocity());

        VelocityCurve curve1 = null;
        VelocityCurve curve2 = new VelocityCurve(s0, s1, v0, v1);
        VelocityCurve curve3 = null;

        // Check: not the same point, constraint point is in range, constraint point's max velocity is less than desired vel
        if (Math.abs(constraintPt1.getS() - p0.getConfigVelocity()) < 1e-10 && !Double.isNaN(constraintPt1.getMaxVelocity()) && curve2.inRange(constraintPt1.getS()) && curve2.getVelocity(constraintPt1.getS()) > constraintPt1.getMaxVelocity()) {
            curve1 = new VelocityCurve(s0, constraintPt1.getS(), v0, constraintPt1.getMaxVelocity());
            curve2 = new VelocityCurve(constraintPt1.getS(), s1, constraintPt1.getMaxVelocity(), v1);
        }
        if (Math.abs(constraintPt2.getS() - p1.getConfigVelocity()) < 1e-10 && !Double.isNaN(constraintPt1.getMaxVelocity()) && curve2.inRange(constraintPt2.getS()) && curve2.getVelocity(constraintPt1.getS()) > constraintPt1.getMaxVelocity()) {
            curve2 = new VelocityCurve(curve2.s0, constraintPt1.getS(), curve2.v0, constraintPt1.getMaxVelocity());
            curve3 = new VelocityCurve(constraintPt1.getS(), s1, constraintPt1.getMaxVelocity(), v1);
        }

        velocityCurves = new ArrayList<>();
        if (curve1 != null) { velocityCurves.add(curve1); }
        velocityCurves.add(curve2);
        if (curve3 != null) { velocityCurves.add(curve3); }
    }

    public String toString() {
        String s = "(s0=" + s0 + ", s1=" + s1 + ", i=" + index + ")\n";
        for (final VelocityCurve vc : velocityCurves) {
            s += vc.toString() + "\n";
        }
        return s;
    }

    public boolean inRange(final double s) {
        if (s0 <= s && s <= s1 + 1e-12) {
            return true;
        } else {
            return false;
        }
    }

//    public double getVelocity(final double s) { // Inefficient, but shouldn't make a huge difference. Right?
//        for (final VelocityCurve vc : velocityCurves) {
//            if (vc.inRange(s)) {
//                return vc.getVelocity(s);
//            }
//        }
//        return Double.NaN; // SOMEBODY GONNA GET HURT REAL BAD
//    }
//
//    public double getAcceleration(final double s) { // Inefficient, but shouldn't make a huge difference. Right?
//        for (final VelocityCurve vc : velocityCurves) {
//            if (vc.inRange(s)) {
//                return vc.acceleration;
//            }
//        }
//        return Double.NaN; // SOMEBODY GONNA GET HURT REAL BAD
//    }

    public double distS(final double s) {
        return s1 - s;
    }
}

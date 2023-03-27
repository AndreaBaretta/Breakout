package org.quixilver8404.breakout.feedforward;

import org.quixilver8404.breakout.util.Vector3;

public class HeadingSegment {
    public final HeadingPoint headingPoint0;
    public final HeadingPoint headingPoint1;
    public final double s0;
    public final double s1;
    public final HeadingPoint.Heading segmentHeading;
    public final int index;
    public final double alpha0;
    public final double alpha1;
    public final double alpha0_;
    public final boolean zeroSegment;

    public HeadingSegment(final HeadingPoint headingPoint0, final HeadingPoint headingPoint1, final int index) {
        this.headingPoint0 = headingPoint0;
        this.headingPoint1 = headingPoint1;
        s0 = headingPoint0.getS();
        s1 = headingPoint1.getS();
        if (headingPoint0.getHeadingState() == headingPoint1.getHeadingState()) {
            segmentHeading = headingPoint0.getHeadingState();
        } else {
            segmentHeading = HeadingPoint.Heading.CUSTOM;
        }
        this.index = index;

        if (headingPoint0.getHeadingState() == HeadingPoint.Heading.FRONT) {
            alpha0 = headingPoint0.getHeading();
        } else if (headingPoint0.getHeadingState() == HeadingPoint.Heading.BACK) {
            alpha0 = Vector3.normalizeAlpha(/*Math.PI + */headingPoint0.getHeading());
        } else {
            alpha0 = headingPoint0.getHeading();
        }
        if (headingPoint1.getHeadingState() == HeadingPoint.Heading.FRONT) {
            alpha1 = headingPoint1.getHeading();
        } else if (headingPoint1.getHeadingState() == HeadingPoint.Heading.BACK) {
            alpha1 = Vector3.normalizeAlpha(/*Math.PI + */headingPoint1.getHeading());
        } else {
            alpha1 = headingPoint1.getHeading();
        }

        if (alpha1 > alpha0) {
            if (alpha1 - alpha0 >= Math.PI) {
                alpha0_ = alpha0 + 2*Math.PI;
            } else {
                alpha0_ = alpha0;
            }
        } else {
            if (alpha0 - alpha1 >= Math.PI) {
                alpha0_ = alpha0 - 2*Math.PI;
            } else {
                alpha0_ = alpha0;
            }
        }

        if (s1 - s0 <= 1e-12) {
            zeroSegment = true;
        } else {
            zeroSegment = false;
        }
    }

    public boolean inRange(final double s) {
        if (s0 <= s && s <= s1 + 1e-12) {
            return true;
        } else {
            return false;
        }
    }

    public Vector3 calcAlpha(final double s, final Vector3 pos) {
        final double alpha;
        if (segmentHeading == HeadingPoint.Heading.FRONT) {
            alpha = pos.theta;
        } else if (segmentHeading == HeadingPoint.Heading.BACK) {
            alpha = pos.theta + Math.PI;
        } else {
            alpha = alpha0 + (alpha1 - alpha0_)*(s - s0)/(s1 - s0);
        }
        return new Vector3(pos.x, pos.y, Vector3.normalizeAlpha(alpha));
    }

    public Vector3 calcAlphaDot(final double s_dot, final Vector3 vel) {
        final double alpha_dot;
        if (segmentHeading == HeadingPoint.Heading.CUSTOM) {
            alpha_dot = s_dot*(alpha1 - alpha0_)/(s1 - s0);
        } else {
            alpha_dot = vel.theta;
        }
        return new Vector3(vel.x, vel.y, alpha_dot);
    }

    public Vector3 calcAlphaDotDot(final double s_dot_dot, final Vector3 acc) {
        final double alpha_dot_dot;
        if (segmentHeading == HeadingPoint.Heading.CUSTOM) {
            alpha_dot_dot = s_dot_dot*(alpha1 - alpha0_)/(s1 - s0);
        } else {
            alpha_dot_dot = acc.theta;
        }
        return new Vector3(acc.x, acc.y, alpha_dot_dot);
    }

    public String toString() {
        final String segmentHeadingString;
        if (segmentHeading == HeadingPoint.Heading.FRONT) {
            segmentHeadingString = "FRONT";
        } else if (segmentHeading == HeadingPoint.Heading.BACK) {
            segmentHeadingString = "BACK";
        } else if (segmentHeading == HeadingPoint.Heading.CUSTOM) {
            segmentHeadingString = "CUSTOM";
        } else {
            segmentHeadingString = "NONE";
        }
        return "(s0=" + s0 + ", s1=" + s1 + ", segmentHeading=" + segmentHeadingString + ", alpha0=" + alpha0 + ", alpha1=" + alpha1 + ", i=" + index + ")";
    }
}

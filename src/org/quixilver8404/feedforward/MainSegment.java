package org.quixilver8404.feedforward;

import org.quixilver8404.util.Config;
import org.quixilver8404.util.Vector3;

public class MainSegment extends Segment {
    public final CircleSegment circleSegment0;
    public final LinearSegment linearSegment;
    public final CircleSegment circleSegment1;

    public final int index;

    protected Segment currentSegment;

    public final AnchorPoint anchorPoint0;
    public final AnchorPoint anchorPoint1;
    public final double alpha0;
    public final double alpha1;
    public final AnchorPoint.Heading heading;
    public final boolean counterClockwise;
    public final double alpha0_;

    MainSegment(final CircleSegment circleSegment0, final LinearSegment linearSegment, final CircleSegment circleSegment1, final int index,
                final AnchorPoint anchorPoint0, final AnchorPoint anchorPoint1) {
        super(circleSegment0.firstPoint, circleSegment1.lastPoint, circleSegment0.s0, Config.MAX_VELOCITY);
        this.circleSegment0 = circleSegment0;
        this.linearSegment = linearSegment;
        this.circleSegment1 = circleSegment1;
        this.index = index;
//        currentSegment = circleSegment0;
        if (!circleSegment0.zeroSegment) {
            currentSegment = circleSegment0;
        } else if (!linearSegment.zeroSegment) {
            currentSegment = linearSegment;
        } else {
            currentSegment = circleSegment1;
        }
        this.anchorPoint0 = anchorPoint0;
        this.anchorPoint1 = anchorPoint1;
        if (anchorPoint0.heading == AnchorPoint.Heading.FRONT) {
            alpha0 = circleSegment0.getPosition(circleSegment0.s0).theta;
        } else if (anchorPoint0.heading == AnchorPoint.Heading.BACK) {
            alpha0 = normalizeAlpha(Math.PI + circleSegment0.getPosition(circleSegment0.s0).theta);
        } else {
            alpha0 = anchorPoint0.customHeading;
        }
        if (anchorPoint1.heading == AnchorPoint.Heading.FRONT) {
            alpha1 = circleSegment1.getPosition(circleSegment1.getEndS()).theta;
        } else if (anchorPoint1.heading == AnchorPoint.Heading.BACK) {
            alpha1 = normalizeAlpha(Math.PI + circleSegment1.getPosition(circleSegment1.getEndS()).theta);
        } else {
            alpha1 = anchorPoint1.customHeading;
        }
        if (anchorPoint0.heading == AnchorPoint.Heading.FRONT && anchorPoint1.heading == AnchorPoint.Heading.FRONT) {
            heading = AnchorPoint.Heading.FRONT;
        } else if (anchorPoint0.heading == AnchorPoint.Heading.BACK && anchorPoint1.heading == AnchorPoint.Heading.BACK) {
            heading = AnchorPoint.Heading.BACK;
        } else {
            heading = AnchorPoint.Heading.CUSTOM;
        }
        if (alpha1 > alpha0) {
            if (alpha1 - alpha0 >= Math.PI) {
                counterClockwise = false;
                alpha0_ = alpha0 + 2*Math.PI;
            } else {
                counterClockwise = true;
                alpha0_ = alpha0;
            }
        } else {
            if (alpha0 - alpha1 >= Math.PI) {
                counterClockwise = true;
                alpha0_ = alpha0 - 2*Math.PI;
            } else {
                counterClockwise = false;
                alpha0_ = alpha0;
            }
        }
    }

    public Vector3 getPosition(final double s) {
        final Vector3 position;
        if (circleSegment0.inRange(s)) {
            position = circleSegment0.getPosition(s);
        } else if (linearSegment.inRange(s)) {
            position = linearSegment.getPosition(s);
        } else if (circleSegment1.inRange(s)) {
//            System.out.println(circleSegment1.zeroSegment);
            position = circleSegment1.getPosition(s);
        } else {
            throw new Error("Out of bounds error that REALLY shouldn't be happening");
        }
//        position = currentSegment.getPosition(s);

        if (heading == AnchorPoint.Heading.FRONT) {
            return position;
        } else if (heading == AnchorPoint.Heading.BACK) {
            return new Vector3(position.x, position.y, position.theta + Math.PI);
        } else {
            final double alpha = alpha0 + (alpha1 - alpha0_)*(s - s0)/(getEndS() - s0);
            return new Vector3(position.x, position.y, normalizeAlpha(alpha));
        }
    }

    public Vector3 getVelocity(final double s, final double s_dot) {
        final Vector3 velocity;
        if (circleSegment0.inRange(s)) {
            velocity = circleSegment0.getVelocity(s, s_dot);
        } else if (linearSegment.inRange(s)) {
            velocity = linearSegment.getVelocity(s, s_dot);
        } else if (circleSegment1.inRange(s)) {
            velocity = circleSegment1.getVelocity(s, s_dot);
        } else throw new Error("Out of bounds error that REALLY shouldn't be happening");
//        velocity = currentSegment.getVelocity(s, s_dot);

        if (heading == AnchorPoint.Heading.CUSTOM) {
            final double alpha_dot = s_dot*(alpha1 - alpha0)/(getEndS() - s0);
            return new Vector3(velocity.x, velocity.y, alpha_dot);
        } else {
            return velocity;
        }
    }

    public Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot) {
        final Vector3 acceleration;
        if (circleSegment0.inRange(s)) {
            acceleration = circleSegment0.getAcceleration(s, s_dot, s_dot_dot);
        } else if (linearSegment.inRange(s)) {
            acceleration = linearSegment.getAcceleration(s, s_dot, s_dot_dot);
        } else if (circleSegment1.inRange(s)) {
            acceleration = circleSegment1.getAcceleration(s, s_dot, s_dot_dot);
        } else throw new Error("Out of bounds error that REALLY shouldn't be happening");
//        acceleration = currentSegment.getAcceleration(s, s_dot, s_dot_dot);

        if (heading == AnchorPoint.Heading.CUSTOM) {
            final double alpha_dot_dot = s_dot_dot*(alpha1 - alpha0)/(getEndS() - s0);
            return new Vector3(acceleration.x, acceleration.y, alpha_dot_dot);
        } else {
            return acceleration;
        }
    }

    public double getTotalS() {
//        return circleSegment0.getTotalS() + linearSegment.getTotalS() + circleSegment1.getTotalS();
        return circleSegment1.getEndS() - s0;
    }

    public double calcMinVelocity() {
        return Config.MAX_VELOCITY;
    }

    public double calcS(final double x, final double y) {
        if (currentSegment == circleSegment0) {
            if (circleSegment0.calcS(x, y) >= circleSegment0.getEndS()) {
                if (linearSegment.zeroSegment) {
                    currentSegment = circleSegment1;
                } else {
                    currentSegment = linearSegment;
                }
            }
            return currentSegment.calcS(x, y);
        } else if (currentSegment == linearSegment) {
            if (linearSegment.calcS(x, y) >= linearSegment.getEndS()) {
                if (!circleSegment1.zeroSegment) {
                    currentSegment = circleSegment1;
                }
            }
            return currentSegment.calcS(x, y);
        } else {
            return currentSegment.calcS(x, y);
        }
    }

    public static double normalizeAlpha(final double alpha) {
        final double sin = Math.sin(alpha);
        final double cos = Math.cos(alpha);
        final double arcsin = Math.asin(sin);
//        System.out.println("sin: " + sin + " cos: " + cos + " arcsin: " + arcsin);
        if (sin >= 0) {
            if (cos >= 0) { //Q1
                return arcsin;
            } else { //Q2
                return Math.PI - arcsin;
            }
        } else {
            if (cos >= 0) { //Q4
                return arcsin + 2*Math.PI;
            } else { //Q3
                return Math.PI - arcsin;
            }
        }
    }

    public static double angleFromSinCos(final double sin, final double cos) {
        final double arcsin = Math.asin(sin);
        if (sin >= 0) {
            if (cos >= 0) { //Q1
                return arcsin;
            } else { //Q2
                return Math.PI - arcsin;
            }
        } else {
            if (cos >= 0) { //Q4
                return arcsin;
            } else { //Q3
                return Math.PI - arcsin;
            }
        }
    }
}

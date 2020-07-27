package com.company.feedforward;

import com.company.simulator.Vector3;

public class CircleSegment extends Segment {

    public final Point2D center;
    public final double r;
    public final double theta0;
    public final double theta1;
    public final boolean counterClockwise;

    public final double theta0_;
    public final double theta1_;

    CircleSegment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0, final Point2D center,
                  final double r, final double theta0, final double theta1, final boolean counterClockwise) {
        super(firstPoint, lastPoint, s0);
        this.center = center;
        this.r = r;
        this.theta0 = theta0;
        this.theta1 = theta1;
        this.counterClockwise = counterClockwise;

        if (counterClockwise) {
            if (theta1 < theta0) {
                theta1_ = 2*Math.PI + theta1;
            } else {
                theta1_ = theta1;
            }
            theta0_ = theta0;
        } else {
            if (theta0 < theta1) {
                theta0_ = 2*Math.PI + theta0;
            } else {
                theta0_ = theta0;
            }
            theta1_ = theta1;
        }
    }

    public Vector3 getPosition(final double s) {
//        final double alpha;
//        final double theta;
//        if (counterClockwise) {
//            alpha = (s/r) + Math.PI;
//            theta = theta0_ + s/r;
//        } else {
//            alpha = (s/r) - Math.PI;
//            theta = theta0_ - s/r;
//        }
//        return new Vector3(
//                center.x + r*Math.cos(s/r),
//                center.y + r*Math.sin(s/r),
//                alpha
//        );
        if (counterClockwise) {
            final double x = center.x + r*Math.cos(theta0_ + (s - s0)/r);
            final double y = center.y + r*Math.sin(theta0_ + (s - s0)/r);
            final double alpha = Math.atan2(y - center.y, x - center.x) + Math.PI/2;
            return new Vector3(
                    x,
                    y,
                    alpha
            );
        } else {
            final double x = center.x + r*Math.cos(theta0_ - (s - s0)/r);
            final double y = center.y + r*Math.sin(theta0_ - (s - s0)/r);
            final double alpha = Math.atan2(y - center.y, x - center.x) - Math.PI/2;
            return new Vector3(
                    x,
                    y,
                    alpha
            );
        }
    }

    public Vector3 getVelocity(final double s, final double s_dot) {
//        return new Vector3(
//                -s_dot*Math.sin(s/r),
//                s_dot*Math.cos(s/r),
//                s_dot/r
//        );
        if (counterClockwise) {
            return new Vector3(
                    -s_dot*Math.sin(theta0_ + s/r),
                    s_dot*Math.cos(theta0_ + s/r),
                    s_dot/r
            );
        } else {
            return new Vector3(
                    s_dot*Math.sin(theta0_ - s/r),
                    -s_dot*Math.cos(theta0_ - s/r),
                    s_dot/r
            );
        }
    }

    public Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot) {
//        return new Vector3(
//                -s_dot_dot*Math.sin(s/r) - Math.pow(s_dot,2)*Math.cos(s/r)/r,
//                s_dot_dot*Math.cos(s/r) - Math.pow(s_dot,2)*Math.sin(s/r)/r,
//                s_dot_dot/r
//        );
        if (counterClockwise) {
            return new Vector3(
                    -s_dot_dot*Math.sin(theta0_ + s/r) - Math.pow(s_dot,2)*Math.cos(theta0_ + s/r)/r,
                    s_dot_dot*Math.cos(theta0_ + s/r) - Math.pow(s_dot,2)*Math.sin(theta0_ + s/r)/r,
                    s_dot_dot/r
            );
        } else {
            return new Vector3(
                    s_dot_dot*Math.sin(theta0_ - s/r) - Math.pow(s_dot,2)*Math.cos(theta0_ - s/r)/r,
                    -s_dot_dot*Math.cos(theta0_ - s/r) - Math.pow(s_dot,2)*Math.sin(theta0_ - s/r)/r,
                    s_dot_dot/r
            );
        }
    }

//    public double getAlpha(final double s) {
//        if (counterClockwise) {
//            return
//        }
//    }
//
//    public double getAngularVelocity(final double s) {
//        return 0;
//    }
//
//    public double getAngularAcceleration(final double s) {
//        return 0;
//    }

    public double getTotalS() {
        if (counterClockwise) {
            return (theta1_ - theta0_)*r;
        } else {
            return (theta0_ - theta1_)*r;
        }
    }
}

package com.company.feedforward;

import com.company.Main;
import com.company.simulator.Vector3;

public class CircleSegment extends MinorSegment {

    public final Point2D center;
    public final double r;
    public final double theta0;
    public final double theta1;
    public final boolean counterClockwise;

    public final double theta0_;
    public final double theta1_;

    CircleSegment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0, final double configVelocity,
                  final Point2D center, final double r, final double theta0, final double theta1, final boolean counterClockwise) {
        super(firstPoint, lastPoint, s0, configVelocity);
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
//        System.out.println("received s: " + s + " counterClockwise: " + counterClockwise);
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
        if (counterClockwise) {
            return new Vector3(
                    -s_dot*Math.sin(theta0_ + (s - s0)/r),
                    s_dot*Math.cos(theta0_ + (s - s0)/r),
                    s_dot/r
            );
        } else {
            return new Vector3(
                    s_dot*Math.sin(theta0_ - (s - s0)/r),
                    -s_dot*Math.cos(theta0_ - (s - s0)/r),
                    -s_dot/r
            );
        }
    }

    public Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot) {
        if (counterClockwise) {
            return new Vector3(
                    -s_dot_dot*Math.sin(theta0_ + (s - s0)/r) - Math.pow(s_dot,2)*Math.cos(theta0_ + (s - s0)/r)/r,
                    s_dot_dot*Math.cos(theta0_ + (s - s0)/r) - Math.pow(s_dot,2)*Math.sin(theta0_ + (s - s0)/r)/r,
                    s_dot_dot/r
            );
        } else {
            return new Vector3(
                    s_dot_dot*Math.sin(theta0_ - (s - s0)/r) - Math.pow(s_dot,2)*Math.cos(theta0_ - (s - s0)/r)/r,
                    -s_dot_dot*Math.cos(theta0_ - (s - s0)/r) - Math.pow(s_dot,2)*Math.sin(theta0_ - (s - s0)/r)/r,
                    -s_dot_dot/r
            );
        }
    }

    public double getTotalS() {
        if (counterClockwise) {
            return (theta1_ - theta0_)*r;
        } else {
            return (theta0_ - theta1_)*r;
        }
    }

    public double calcMinVelocity() {
        return Math.sqrt(r * Config.MAX_ACCELERATION);
    }

    public double calcS(final double x, final double y) {
        final double gamma = MainSegment.normalizeAlpha(Math.atan2(y - center.y, x - center.x));
        final double theta;
        if (counterClockwise) {
//            if (gamma < theta0) {
//                theta = (gamma + 2*Math.PI) - theta0;
//            } else {
//                theta = gamma - theta0;
//                System.out.println("theta: " + theta);
//            }
//            theta = gamma - MainSegment.normalizeAlpha(theta0);
            final double sin = Math.sin(gamma)*Math.cos(theta0) - Math.cos(gamma)*Math.sin(theta0);
            final double cos = Math.cos(gamma)*Math.cos(theta0) + Math.sin(gamma)*Math.sin(theta0);
            theta = MainSegment.angleFromSinCos(sin, cos);
        } else {

//            if (gamma > theta0) {
//                theta = theta0 - (gamma - 2*Math.PI);
//            } else {
//                theta = theta0 - gamma;
//            }
//            theta = MainSegment.normalizeAlpha(theta0) - gamma;
            final double sin = Math.cos(gamma)*Math.sin(theta0) - Math.sin(gamma)*Math.cos(theta0);
            final double cos = Math.cos(gamma)*Math.cos(theta0) + Math.sin(gamma)*Math.sin(theta0);
            theta = MainSegment.angleFromSinCos(sin, cos);
        }

//        System.out.println("gamma: " + gamma + " theta: " + MainSegment.normalizeAlpha(theta0) + " s: " + (theta*r + s0));
//        if (theta*r + s0 <= s0) {
//            System.out.println("This was useful");
//            System.out.println("gamma: " + gamma + " theta: " + theta + " r: " + r + " s0: " + s0 + " s: " + (theta*r + s0));
//            return s0;
//        }
//        System.out.println("gamma: " + gamma + " theta0: " + theta0 + " theta: " + theta + " r: " + r + " s0: " + s0 + " s: " + (theta*r + s0));
        return theta*r + s0;
    }
}

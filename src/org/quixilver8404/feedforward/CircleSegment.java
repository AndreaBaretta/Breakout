package org.quixilver8404.feedforward;

import org.quixilver8404.util.Config;
import org.quixilver8404.util.Vector3;

import java.util.List;

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
//        System.out.println("Theta0 in circle: " + theta0);
//        System.out.println("Theta1 in circle: " + theta1);
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
        setPointSegment();
    }

    public Vector3 getPosition(final double s) {
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

    public double getMinVelocity() {
//        System.out.println("In circle: r=" + r + " max_a=" + Config.MAX_ACCELERATION);
        return Math.min(Math.sqrt(r * Config.MAX_ACCELERATION), Config.MAX_SAFE_VELOCITY*Config.MAX_VELOCITY);
    }

    public double calcS(final double x, final double y) {
        final double gamma = MainSegment.normalizeAlpha(Math.atan2(y - center.y, x - center.x));
        final double theta;
        if (counterClockwise) {
            final double sin = Math.sin(gamma)*Math.cos(theta0) - Math.cos(gamma)*Math.sin(theta0);
            final double cos = Math.cos(gamma)*Math.cos(theta0) + Math.sin(gamma)*Math.sin(theta0);
            theta = MainSegment.normalizeAlpha(MainSegment.angleFromSinCos(sin, cos));
        } else {
            final double sin = Math.cos(gamma)*Math.sin(theta0) - Math.sin(gamma)*Math.cos(theta0);
            final double cos = Math.cos(gamma)*Math.cos(theta0) + Math.sin(gamma)*Math.sin(theta0);
            theta = MainSegment.normalizeAlpha(MainSegment.angleFromSinCos(sin, cos));
        }

        return theta*r + s0;
    }
}

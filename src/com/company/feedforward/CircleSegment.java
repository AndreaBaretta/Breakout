package com.company.feedforward;

import com.company.simulator.Vector3;

public class CircleSegment extends Segment {

    public final Point2D center;
    public final double r;
    public final double theta0;
    public final double theta1;
    public final boolean reverse;

    CircleSegment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final Point2D center, final double r,
                  final double theta0, final double theta1, final boolean reverse) {
        super(firstPoint, lastPoint);
        this.center = center;
        this.r = r;
        this.theta0 = theta0;
        this.theta1 = theta1;
        this.reverse = reverse;
    }

    public Vector2 getPosition(final double s) {
        return new Vector2(0,0);
    }

    public Vector2 getVelocity(final double s) {
        return new Vector2(0,0);
    }

    public Vector2 getAcceleration(final double s) {
        return new Vector2(0,0);
    }

    public double getAlpha(final double s) {
        return 0;
    }

    public double getAngularVelocity(final double s) {
        return 0;
    }

    public double getAngularAcceleration(final double s) {
        return 0;
    }
}

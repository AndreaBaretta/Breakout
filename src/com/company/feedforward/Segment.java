package com.company.feedforward;

import com.company.simulator.Vector3;

public abstract class Segment {
    final ConnectionPoint firstPoint;
    final ConnectionPoint lastPoint;

    Segment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint) {
        this.firstPoint = firstPoint;
        this.lastPoint = lastPoint;
        firstPoint.setNextSegment(this);
        lastPoint.setPrevSegment(this);
    }

    public abstract Vector2 getPosition(final double s);

    public abstract Vector2 getVelocity(final double s);

    public abstract Vector2 getAcceleration(final double s);

    public abstract double getAlpha(final double s);

    public abstract double getAngularVelocity(final double s);

    public abstract double getAngularAcceleration(final double s);
}

package com.company.feedforward;

public abstract class MinorSegment extends Segment {
    public final ConnectionPoint firstPoint;
    public final ConnectionPoint lastPoint;
    MinorSegment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0, final double configVelocity) {
        super(firstPoint, lastPoint, s0, configVelocity);
        this.firstPoint = firstPoint;
        this.lastPoint = lastPoint;
        firstPoint.setNextSegment(this);
        lastPoint.setPrevSegment(this);
    }
}